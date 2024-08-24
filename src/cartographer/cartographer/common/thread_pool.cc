/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/thread_pool.h"

#ifndef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

#include "absl/memory/memory.h"
#include "cartographer/common/task.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// ִ�д���� task��Execute()����
void ThreadPoolInterface::Execute(Task* task) { task->Execute(); }

// ִ�д���� task��SetThreadPool()����
void ThreadPoolInterface::SetThreadPool(Task* task) {
  task->SetThreadPool(this);
}

// ���ݴ��������, �����̳߳صĹ���, DoWork()������ʼ��һ��ʼ��ִ�е�forѭ��
ThreadPool::ThreadPool(int num_threads) {
  CHECK_GT(num_threads, 0) << "ThreadPool requires a positive num_threads!";
  absl::MutexLock locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

// ֻ�еȴ� pool_ �������е��߳�(join�ǵȴ�ֱ���߳̽���),ThreadPool�����������
ThreadPool::~ThreadPool() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK(running_);
    running_ = false;
  }
  for (std::thread& thread : pool_) {
    thread.join();
  }
}

// task��������������, ���Խ�task�����ִ������Ķ���task_queue_����
void ThreadPool::NotifyDependenciesCompleted(Task* task) {
  absl::MutexLock locker(&mutex_);

  // �ҵ�task������
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());

  // ���뵽���������
  task_queue_.push_back(it->second);
  // ��δ׼���õ����������ɾ��task
  tasks_not_ready_.erase(it);
}

// ��task���뵽tasks_not_ready_������, ��ִ��task��SetThreadPool()����
std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
  std::shared_ptr<Task> shared_task;
  {
    absl::MutexLock locker(&mutex_);
    auto insert_result =
        tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));

    // map::insert() �᷵��pair<map::iterator,bool> ����, 
    // ��һ��ֵΪ������, �ڶ���ֵΪ��������Ƿ�ɹ�
    CHECK(insert_result.second) << "Schedule called twice";
    shared_task = insert_result.first->second;
  }
  SetThreadPool(shared_task.get());
  return shared_task;
}

// ��ʼһ����ֹͣ��forѭ��, ���������в�Ϊ��, ��ִ�е�һ��task
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif

  const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return !task_queue_.empty() || !running_;
  };

  // ʼ��ִ��, ֱ��running_Ϊfalseʱִֹͣ��
  for (;;) {
    std::shared_ptr<Task> task;
    {
      absl::MutexLock locker(&mutex_);
      mutex_.Await(absl::Condition(&predicate));

      // map_builder.lua�����õ��߳���, 4���̴߳���ͬһ��task_queue_
      // ���������в�Ϊ��, �Ǿ�ȡ����һ��task
      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      } else if (!running_) {
        return;
      }
    }
    CHECK(task);
    CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);

    // ִ��task
    Execute(task.get());
  }
}

}  // namespace common
}  // namespace cartographer
