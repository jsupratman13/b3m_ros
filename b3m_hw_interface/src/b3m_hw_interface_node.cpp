/*********************************************************************
 * Copyright (c) 2023
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <vector>
#include <thread>
#include <mutex>

#include <queue>
#include <functional>
#include <condition_variable>

class ThreadPool
{
public:
  explicit ThreadPool(size_t n) : stop_(false)
  {
    for (size_t i = 0; i < n; ++i)
    {
      workers_.emplace_back([this]() {
        while (true)
        {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            condition_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty())
              return;
            task = std::move(tasks_.front());
            tasks_.pop();
          }
          task();
        }
      });
    }
  }

  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      stop_ = true;
    }
    condition_.notify_all();
    for (std::thread& worker : workers_)
      worker.join();
  }

  void enqueue(std::function<void()> func)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      tasks_.push(std::move(func));
    }
    condition_.notify_one();
  }

  void waitAll()
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    condition_.wait(lock, [this] { return tasks_.empty(); });
  }

  // For each round, we can track #outstanding jobs to allow waitAll().
  // Instead, let's do it per read/write with local barriers.

private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;
  bool stop_;
};

class ParallelCombinedRobotHW : public combined_robot_hw::CombinedRobotHW
{
private:
  ThreadPool pool_;

public:
  ParallelCombinedRobotHW() : pool_(std::thread::hardware_concurrency())
  {
    // Initialize the thread pool with the number of hardware threads available
    ROS_INFO_STREAM("Thread pool initialized with " << std::thread::hardware_concurrency() << " threads.");
  }
  bool init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) override
  {
    root_nh_ = nh;
    robot_hw_nh_ = nh_private;
    std::vector<std::string> robots;
    std::string param_name = "robot_hardware";
    if (!robot_hw_nh_.getParam(param_name, robots))
    {
      ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh_private.getNamespace()
                                          << ").");
      return false;
    }
    std::vector<std::thread> threads;
    std::vector<bool> init_results(robots.size(), true);
    std::mutex log_mutex;
    for (size_t i = 0; i < robots.size(); ++i)
    {
      threads.emplace_back([this, &robots, &init_results, &log_mutex, i]() {
        ROS_INFO_STREAM("Initializing RobotHW for: " << robots[i]);
        bool success = loadRobotHW(robots[i]);
        if (!success)
        {
          std::lock_guard<std::mutex> lock(log_mutex);
          ROS_ERROR_STREAM("Failed to load RobotHW for: " << robots[i]);
          init_results[i] = false;
        }
        else
        {
          std::lock_guard<std::mutex> lock(log_mutex);
          ROS_INFO_STREAM("Successfully loaded RobotHW for: " << robots[i]);
        }
      });
    }
    for (auto& thread : threads)
    {
      if (thread.joinable())
        thread.join();
    }
    // Check if all hardware interfaces were successfully initialized
    for (const auto& result : init_results)
    {
      if (!result)
        return false;
    }
    return true;
  }
  void read(const ros::Time& time, const ros::Duration& period) override
  {
    std::atomic<size_t> jobs_left(robot_hw_list_.size());
    std::condition_variable cv;
    std::mutex mtx;

    for (size_t i = 0; i < robot_hw_list_.size(); ++i)
    {
      pool_.enqueue([&, i]() {
        robot_hw_list_[i]->read(time, period);
        if (--jobs_left == 0)
        {
          std::lock_guard<std::mutex> lock(mtx);
          cv.notify_one();
        }
      });
    }

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&] { return jobs_left == 0; });
  }
  void write(const ros::Time& time, const ros::Duration& period) override
  {
    std::atomic<size_t> jobs_left(robot_hw_list_.size());
    std::condition_variable cv;
    std::mutex mtx;

    for (size_t i = 0; i < robot_hw_list_.size(); ++i)
    {
      pool_.enqueue([&, i]() {
        robot_hw_list_[i]->write(time, period);
        if (--jobs_left == 0)
        {
          std::lock_guard<std::mutex> lock(mtx);
          cv.notify_one();
        }
      });
    }

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&] { return jobs_left == 0; });
  }
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list) override
  {
    bool success = true;
    std::vector<std::thread> threads;
    std::vector<bool> results(robot_hw_list_.size(), true);
    for (size_t i = 0; i < robot_hw_list_.size(); ++i)
    {
      threads.emplace_back([&, i]() {
        std::list<hardware_interface::ControllerInfo> filtered_start_list;
        std::list<hardware_interface::ControllerInfo> filtered_stop_list;

        filterControllerList(start_list, filtered_start_list, robot_hw_list_[i]);
        filterControllerList(stop_list, filtered_stop_list, robot_hw_list_[i]);

        if (!robot_hw_list_[i]->prepareSwitch(filtered_start_list, filtered_stop_list))
        {
          results[i] = false;
        }
      });
    }
    for (auto& thread : threads)
    {
      thread.join();
    }
    for (const auto& res : results)
    {
      if (!res)
      {
        success = false;
        break;
      }
    }

    return success;
  }
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override
  {
    std::vector<std::thread> threads;
    threads.reserve(robot_hw_list_.size());
    for (auto& robot_hw : robot_hw_list_)
    {
      threads.emplace_back([&]() {
        std::list<hardware_interface::ControllerInfo> filtered_start_list;
        std::list<hardware_interface::ControllerInfo> filtered_stop_list;

        filterControllerList(start_list, filtered_start_list, robot_hw);
        filterControllerList(stop_list, filtered_stop_list, robot_hw);

        robot_hw->doSwitch(filtered_start_list, filtered_stop_list);
      });
    }
    for (auto& thread : threads)
    {
      thread.join();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "b3m_hw_interface_node");
  ros::NodeHandle nh, nh_private("~");

  ParallelCombinedRobotHW hw;
  if (!hw.init(nh, nh_private))
  {
    ROS_FATAL_STREAM("Initializing hardware interfaces failed");
    ros::shutdown();
    return 1;
  }
  ROS_INFO_STREAM("Initializing hardware interfaces succeeded, starting hardware interface");

  controller_manager::ControllerManager cm(&hw, nh);
  ros::AsyncSpinner spinner(4);
  ros::Rate rate(50);  // 50Hz update rate

  spinner.start();
  while (ros::ok())
  {
    hw.read(ros::Time::now(), rate.expectedCycleTime());
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    hw.write(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
