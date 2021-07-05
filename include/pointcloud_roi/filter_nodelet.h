#ifndef FILTER_NODELET_H
#define FILTER_NODELET_H

#include <nodelet/nodelet.h>

namespace pointcloud_roi
{

template<class Filter>
class FilterNodelet : public nodelet::Nodelet
{
public:
  FilterNodelet() : Nodelet() {}

private:
  std::unique_ptr<Filter> filter;

  virtual void onInit()
  {
    filter.reset(new Filter(getPrivateNodeHandle()));
  }
};

} // namespace pointcloud_roi

#endif // FILTER_NODELET_H
