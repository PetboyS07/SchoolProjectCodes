#ifndef THREAD_UTILITIES_H
#define THREAD_UTILITIES_H

#include <mutex>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//thread utilities
namespace thut
{

struct sharedGrabData_s
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;
  int frame;
};

template <typename T>
class IQueue
{
public:

  virtual ~IQueue() = default;

  virtual void PushBack(const T& value) = 0;
  virtual T PopFront() = 0;
  virtual T PopBack() = 0;
  virtual T Back() const = 0;
  virtual T Front() const = 0;
  virtual size_t Size() const = 0;
  virtual size_t RealSize() const = 0;
  virtual bool IsEmpty() const = 0;
  virtual bool HasData() const = 0;
  virtual void Clear() = 0;
};



template <typename T, unsigned int Capacity = 30U>
class CLinearDataQueue : public IQueue<T>
{
public:

  CLinearDataQueue() : m_index(0U)
  {
    static_assert(Capacity > 0, "This queue must be able to hold at least 1 item");
    m_queue.reserve(Capacity);
  }

  CLinearDataQueue(const CLinearDataQueue& src) = delete;
  CLinearDataQueue(CLinearDataQueue&& src) = delete;

  CLinearDataQueue& operator= (const CLinearDataQueue& src) = delete;
  CLinearDataQueue& operator= (CLinearDataQueue&& src) = delete;

  void PushBack(const T& value)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push_back(value);
  }

  T PopFront()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    T out = m_queue.at(m_index);

    ++m_index;
    CheckCapacity();

    return out;
  }

  T PopBack()
  {
    T out = Back();

    std::lock_guard<std::mutex> lock(m_mutex);

    m_queue.pop_back();
    CheckCapacity();
    return out;
  }

  T Front() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    T out = m_queue.at(m_index);
    return out;
  }

  T Back() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    T out = m_queue.back();
    return out;
  }

  size_t Size() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return (m_queue.size() - m_index);
  }

  size_t RealSize() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size();
  }

  bool IsEmpty() const
  {
    return (0 == Size());
  }

  bool HasData() const
  {
    return (Size() > 0);
  }

  void Clear()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_index = 0;
    m_queue.clear();
  }
private:
  mutable std::mutex m_mutex;
  unsigned int       m_index;

  std::vector<T>     m_queue;

  void CheckCapacity()
  {
    if (m_index > (m_queue.size() - 1U))
    {
      m_queue.erase(m_queue.begin(), m_queue.begin() + m_index);
      m_index = 0;
    }
  }

};


extern CLinearDataQueue<sharedGrabData_s, 20> grabQueue;


} // thut


#endif // THREAD_UTILITIES_H
