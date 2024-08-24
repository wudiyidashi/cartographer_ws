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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

/**
 * @brief ���һ�����ݶ���,������ص����� CollatedTrajectoryBuilder::HandleCollatedSensorData
 * 
 * @param[in] queue_key �켣id��topic����
 * @param[in] callback void(std::unique_ptr<Data> data) �͵ĺ���
 * �����callback�Ѿ��Ƕ�Ӧsensor_id��callback��
 */
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

// ��queue_key��Ӧ��Queue��finished���ó�true
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";

  auto& queue = it->second;
  CHECK(!queue.finished);

  queue.finished = true;
  Dispatch();
}

// �����ݶ������������
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  // ���queue_key����queues_��, �ͺ���data
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }

  // �����ݶ������������
  it->second.queue.Push(std::move(data));

  // ���������ݵķַ�����
  Dispatch();
}

// �����д���δ���״̬�����ݶ��б��Ϊ���״̬
void OrderedMultiQueue::Flush() {
  // �ҵ�����unfinished�����ݶ���
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  // ��unfinished_queues���Ϊ���״̬
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}

// ���������Ķ��е�QueueKey
QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/**
 * @brief ���������ݶ����е����ݸ���ʱ�����δ���ص�����(���ݷַ�)
 * 
 * 3���˳����:
 * �˳�����1 ĳ����������ݶ���Ϊ��ͬʱ�ֲ������״̬, ���˳�
 * �˳�����2 ֻ�ж����queues_Ϊ��, ���˳�
 * �˳�����3 ���ݶ��������ݵĸ���ֻ��1��,�ֲ������״̬,����ȷ��״̬, �����˳�
 */
void OrderedMultiQueue::Dispatch() {
  while (true) {
    /*
      queues_: 
        (0, scan): {      4,     }
        (0, imu):  {1,  3,   5,  }
        (0, odom): {  2,       6,}
    */
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;

    // Step: 1 �������е����ݶ���, �ҵ��������ݶ��еĵ�һ��������ʱ�����ϵ�һ������
    for (auto it = queues_.begin(); it != queues_.end();) {

      // c++11: auto*(ָ������˵����), auto&(��������˵����), auto &&(��ֵ����)

      // ��ȡ��ǰ������ʱ�����ϵ�һ����һ������
      const auto* data = it->second.queue.Peek<Data>();

      if (data == nullptr) {
        // ��������Ѿ�����finished״̬��, ��ɾ���������
        if (it->second.finished) {
          queues_.erase(it++);
          continue;
        }
        // �˳�����1: ĳ����������ݶ���Ϊ��ͬʱ�ֲ������״̬, �����˳�, ����log�����Ϊ������
        CannotMakeProgress(it->first);
        return;
      }

      // ��һ�ν��е��������data��ʱ���next_data��ʱ��С(������)
      // �͸���next_data, �����浱ǰ��������ݶ����Լ�queue_key
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }

      // ���ݵ�ʱ������ǰ�˳���, �ͱ���
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      
      ++it;
    } // end for

    // �˳�����2: ֻ�ж����queues_Ϊ��, �ſ���next_data==nullptr
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    // ������ǻ�û��Ϊ����켣�����κ�����, �������켣�����ж���, ֱ���ﵽһ����ͬ�Ŀ�ʼʱ��
    
    // Step: 2 ��ȡ��Ӧ�켣id���������ݶ����е���С��ͬʱ���, ��Ϊ�켣��ʼ��ʱ��
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    // Step: 3 �� next_queue ��ʱ�����ϵ�һ�����ݴ���ص��������д��� 

    // ��������, ����ʱ�䶼�ᳬ��common_start_time��
    if (next_data->GetTime() >= common_start_time) {
      // Happy case, we are beyond the 'common_start_time' already.
      // ���·ַ����ݵ�ʱ��
      last_dispatched_time_ = next_data->GetTime();
      // �����ݴ��� callback() �������д���,����������ݴ����ݶ�����ɾ��
      next_queue->callback(next_queue->queue.Pop());
    } 
    // ����ʱ��С��common_start_time,ͬʱ���ݶ������ݵĸ���С��2,ֻ��1�����ݵ���� ����
    else if (next_queue->queue.Size() < 2) {
      // �˳�����3: ���ݶ������ݵĸ�����,�ֲ������״̬, ����ȷ�����ڵ�����ɶ���, �����˳��Ժ��ٴ���
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      } 
      // �������״̬��, �����ݴ��� callback() ����������󼸸����ݵĴ���
      // ���·ַ����ݵ�ʱ��,�����ݴ��� callback() ���д���,����������ݴ����ݶ�����ɾ��
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } 
    // ����ʱ��С��common_start_time,ͬʱ���ݶ������ݵĸ������ڵ���2��
    else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.

      // ֻ����������common_start_time��ǰһ������, ������������ݻᱻ������
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        // ���·ַ����ݵ�ʱ��,�����ݴ��� callback() ���д���
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

// ���queue_keyΪ������,������������log,�ȵ��������
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  // ���queue_keyΪ������
  blocker_ = queue_key;
  for (auto& entry : queues_) {
    // queue_key��Ӧ�����ݶ���Ϊ��,��ĳһ�����������ݶ��е������Ѿ�����kMaxQueueSize��
    // ������, ���б���
    if (entry.second.queue.Size() > kMaxQueueSize) {
      // �ڸ�����1��61��121�����α�ִ�е�ʱ��, ��¼��־��Ϣ
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;

      // [ WARN] [1628516438.493835120, 1606808659.273453929]: W0809 21:40:38.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      // [ WARN] [1628516439.089736487, 1606808659.869309184]: W0809 21:40:39.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      return;
    }
  }
}

/**
 * @brief �ҵ����ݶ������е�һ֡�����ʱ��(��ͬʱ��)
 * ����ĳ��id�Ĺ켣�� common_start_time ֻ�����һ��
 * 
 * @param[in] trajectory_id �켣id
 * @return common::Time �������ݶ������е�һ֡�����ʱ��
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {

  // c++11: map::emplace() ���ص� pair ����
  // pair �ĳ�Ա���� first ��һ��ָ�����Ԫ�ػ���ֹ�����Ԫ�صĵ�����
  // ��Ա���� second �Ǹ�����ֵ, ��ʾ�Ƿ����ɹ�, ������Ԫ�ص������Ѿ����ڲ����ʧ��,����false
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;

  // �������ɹ��˾��ҵ�ʱ������Ķ�common_start_time���и���, ʧ���˾Ͳ�����
  // ֻ���ڹ켣��ʼʱ����ɹ�һ��
  if (emplace_result.second) {
    // �ҵ�����켣��,�������ݶ��������ݵ�ʱ������ ��ʱ���
    // ִ�е�����ʱ, ���е����ݶ��ж���ֵ��, ��Ϊûֵ�������Dispatch()����ǰ������
    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";

    // [ INFO] [1628516134.243770381, 1606808649.533687125]: I0809 21:35:34.000000  8604 ordered_multi_queue.cc:264] All sensor data for trajectory 0 is available starting at '637424054495384530'.

  }

  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer
