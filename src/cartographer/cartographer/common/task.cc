/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/common/task.h"

namespace cartographer {
namespace common {

Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  if (state_ != NEW && state_ != COMPLETED) {
    LOG(WARNING) << "Delete Task between dispatch and completion.";
  }
}

// ���ر�Task��ǰ״̬ 
Task::State Task::GetState() {
  absl::MutexLock locker(&mutex_);
  return state_;
}

// ���ñ�Task��Ҫִ�е����� �������� 
// ״̬: NEW
void Task::SetWorkItem(const WorkItem& work_item) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;
}

// c++11: std::weak_ptr weak_ptr�����Ϊ��shared_ptr��ͬ����, 
// ���Դ�һ��shared_ptr������һ��weak_ptr������, �����Դ�Ĺ۲�Ȩ
// ��weak_ptrû�й�����Դ, ���Ĺ��첻������ָ�����ü���������.
// ͬ��, ��weak_ptr����ʱҲ���ᵼ�����ü����ļ���, ��ֻ��һ�������ع۲���.
// weak_ptrû������operator*��->, ���������, ��Ϊ��������ָ��, ���ܲ�����Դ, ����������ԭ��
// ��������ʹ��һ���ǳ���Ҫ�ĳ�Ա����lock()�ӱ��۲��shared_ptr���һ�����õ�shared_ptr����, �Ӷ�������Դ.

// Ϊ�������������
void Task::AddDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, NEW);
    // ���ָ��ָ��ɹ���ȡ����
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  
  if (shared_dependency) {
    // ����task���뵽shared_dependency�ļ���dependent_tasks_��
    shared_dependency->AddDependentTask(this);
  }
}

// ���̳߳��뱾������������, ���û��δ��ɵ�����, ������̳߳ؿ��Խ���������뵽ִ�ж�����
// ״̬: NEW -> DISPATCHED || NEW -> DISPATCHED -> DEPENDENCIES_COMPLETED
void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);

  // ������״̬����Ϊ DISPATCHED
  state_ = DISPATCHED;

  // ��thread_pool_to_notify_ָ��ָ�����thread_pool
  thread_pool_to_notify_ = thread_pool;

  // �����Taskû��δ��ɵ�����, ��֪ͨ�̳߳ؿ��Խ���������뵽ִ�ж�����
  if (uncompleted_dependencies_ == 0) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

// ���������Task��Task
void Task::AddDependentTask(Task* dependent_task) {
  absl::MutexLock locker(&mutex_);

  // �����Task�����, �Ǿ�֪ͨ����dependent_task
  if (state_ == COMPLETED) {
    dependent_task->OnDependenyCompleted();
    return;
  }
  // ��������������������set��
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

// ���������������������, ���Խ���������뵽�̳߳صĴ������б�����
// ״̬: DISPATCHED -> DEPENDENCIES_COMPLETED
void Task::OnDependenyCompleted() {
  absl::MutexLock locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  // �����������һ
  --uncompleted_dependencies_;
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

// ִ�б�����, Ҳ���Ǵ���ĺ���work_item_
// ״̬: DEPENDENCIES_COMPLETED -> RUNNING -> COMPLETED
void Task::Execute() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
    state_ = RUNNING;
  }

  // Execute the work item.
  if (work_item_) {
    work_item_();
  }

  absl::MutexLock locker(&mutex_);
  state_ = COMPLETED;

  // ֪ͨ�������������������, ������ִ������
  for (Task* dependent_task : dependent_tasks_) {
    dependent_task->OnDependenyCompleted();
  }
}

}  // namespace common
}  // namespace cartographer
