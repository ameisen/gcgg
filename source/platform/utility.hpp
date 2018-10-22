#pragma once

#include <type_traits>

namespace gcgg
{

  /* vector only for now. TODO. */
  template <typename T, typename U>
  void fast_erase_element(std::vector<T> & __restrict container, const U & __restrict element)
  {
    //static_assert(std::is_convertible_v<T, U>, "fast_erase_element type must be copmatible with container type");
    
    const size_t container_size = container.size();
    if (container_size == 1)
    {
      if (container[0] == element)
      {
        container.clear();
      }
    }
    else
    {
      for (size_t i = 0; i < container_size; ++i)
      {
        if (container[i] == element)
        {
          if (i != container_size - 1)
          {
            container[i] = std::move(container[container_size - 1]);
          }
          container.pop_back();
          return;
        }
      }
    }
  }

  /* vector only for now. TODO. */
  template <typename T>
  void fast_erase_index(std::vector<T> & __restrict container, size_t index)
  {
    const size_t container_size = container.size();

    if (container_size == 1)
    {
      container.clear();
    }
    else
    {
      if (index != container_size - 1)
      {
        container[index] = std::move(container[container_size - 1]);
      }
      container.pop_back();
    }
  }

  /* vector only for now. TODO. */
  template <typename T>
  void fast_erase_iterator(std::vector<T> & __restrict container, typename std::vector<T>::iterator & __restrict iter)
  {
    const size_t index = iter - container.begin();
    fast_erase_index(container, index);
  }
}