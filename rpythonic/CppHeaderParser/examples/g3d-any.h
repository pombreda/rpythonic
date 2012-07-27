namespace std __attribute__ ((__visibility__ ("default"))) {

  template <typename _Tp, typename _Alloc>
    deque<_Tp, _Alloc>&
    deque<_Tp, _Alloc>::
    operator=(__const__ deque& __x)
    {
      __const__ size_type __len = size();
      if (&__x != this)
 {
   if (__len >= __x.size())
     _M_erase_at_end(std::copy(__x.begin(), __x.end(),
          this->_M_impl._M_start));
   else
     {
       const_iterator __mid = __x.begin() + difference_type(__len);
       std::copy(__x.begin(), __mid, this->_M_impl._M_start);
       insert(this->_M_impl._M_finish, __mid, __x.end());
     }
 }
      return *this;
    }
# 118 "/usr/include/c++/4.5/bits/deque.tcc" 3
  template <typename _Tp, typename _Alloc>
    typename deque<_Tp, _Alloc>::iterator
    deque<_Tp, _Alloc>::
    insert(iterator __position, __const__ value_type& __x)
    {
      if (__position._M_cur == this->_M_impl._M_start._M_cur)
 {
   push_front(__x);
   return this->_M_impl._M_start;
 }
      else if (__position._M_cur == this->_M_impl._M_finish._M_cur)
 {
   push_back(__x);
   iterator __tmp = this->_M_impl._M_finish;
   --__tmp;
   return __tmp;
 }
      else
        return _M_insert_aux(__position, __x);
    }
# 163 "/usr/include/c++/4.5/bits/deque.tcc" 3
  template <typename _Tp, typename _Alloc>
    typename deque<_Tp, _Alloc>::iterator
    deque<_Tp, _Alloc>::
    erase(iterator __position)
    {
      iterator __next = __position;
      ++__next;
      __const__ difference_type __index = __position - begin();
      if (static_cast<size_type>(__index) < (size() >> 1))
 {
   if (__position != begin())
     std::copy_backward(begin(), __position, __next);
   pop_front();
 }
      else
 {
   if (__next != end())
     std::copy(__next, end(), __position);
   pop_back();
 }
      return begin() + __index;
    }

  template <typename _Tp, typename _Alloc>
    typename deque<_Tp, _Alloc>::iterator
    deque<_Tp, _Alloc>::
    erase(iterator __first, iterator __last)
    {
      if (__first == begin() && __last == end())
 {
   clear();
   return end();
 }
      else
 {
   __const__ difference_type __n = __last - __first;
   __const__ difference_type __elems_before = __first - begin();
   if (static_cast<size_type>(__elems_before) <= (size() - __n) / 2)
     {
       if (__first != begin())
  std::copy_backward(begin(), __first, __last);
       _M_erase_at_begin(begin() + __n);
     }
   else
     {
       if (__last != end())
  std::copy(__last, end(), __first);
       _M_erase_at_end(end() - __n);
     }
   return begin() + __elems_before;
 }
    }

  template <typename _Tp, class _Alloc>
    template <typename _InputIterator>
      void
      deque<_Tp, _Alloc>::
      _M_assign_aux(_InputIterator __first, _InputIterator __last,
      std::input_iterator_tag)
      {
        iterator __cur = begin();
        for (; __first != __last && __cur != end(); ++__cur, ++__first)
          *__cur = *__first;
        if (__first == __last)
          _M_erase_at_end(__cur);
        else
          insert(end(), __first, __last);
      }

  template <typename _Tp, typename _Alloc>
    void
    deque<_Tp, _Alloc>::
    _M_fill_insert(iterator __pos, size_type __n, __const__ value_type& __x)
    {
      if (__pos._M_cur == this->_M_impl._M_start._M_cur)
 {
   iterator __new_start = _M_reserve_elements_at_front(__n);
   try
     {
       std::__uninitialized_fill_a(__new_start, this->_M_impl._M_start,
       __x, _M_get_Tp_allocator());
       this->_M_impl._M_start = __new_start;
     }
   catch(...)
     {
       _M_destroy_nodes(__new_start._M_node,
          this->_M_impl._M_start._M_node);
       throw;
     }
 }
      else if (__pos._M_cur == this->_M_impl._M_finish._M_cur)
 {
   iterator __new_finish = _M_reserve_elements_at_back(__n);
   try
     {
       std::__uninitialized_fill_a(this->_M_impl._M_finish,
       __new_finish, __x,
       _M_get_Tp_allocator());
       this->_M_impl._M_finish = __new_finish;
     }
   catch(...)
     {
       _M_destroy_nodes(this->_M_impl._M_finish._M_node + 1,
          __new_finish._M_node + 1);
       throw;
     }
 }
      else
        _M_insert_aux(__pos, __n, __x);
    }

  template <typename _Tp, typename _Alloc>
    void
    deque<_Tp, _Alloc>::
    _M_fill_initialize(__const__ value_type& __value)
    {
      _Map_pointer __cur;
      try
        {
          for (__cur = this->_M_impl._M_start._M_node;
        __cur < this->_M_impl._M_finish._M_node;
        ++__cur)
            std::__uninitialized_fill_a(*__cur, *__cur + _S_buffer_size(),
     __value, _M_get_Tp_allocator());
          std::__uninitialized_fill_a(this->_M_impl._M_finish._M_first,
          this->_M_impl._M_finish._M_cur,
          __value, _M_get_Tp_allocator());
        }
      catch(...)
        {
          std::_Destroy(this->_M_impl._M_start, iterator(*__cur, __cur),
   _M_get_Tp_allocator());
          throw;
        }
    }

  template <typename _Tp, typename _Alloc>
    template <typename _InputIterator>
      void
      deque<_Tp, _Alloc>::
      _M_range_initialize(_InputIterator __first, _InputIterator __last,
                          std::input_iterator_tag)
      {
        this->_M_initialize_map(0);
        try
          {
            for (; __first != __last; ++__first)
              push_back(*__first);
          }
        catch(...)
          {
            clear();
            throw;
          }
      }

  template <typename _Tp, typename _Alloc>
    template <typename _ForwardIterator>
      void
      deque<_Tp, _Alloc>::
      _M_range_initialize(_ForwardIterator __first, _ForwardIterator __last,
                          std::forward_iterator_tag)
      {
        __const__ size_type __n = std::distance(__first, __last);
        this->_M_initialize_map(__n);

        _Map_pointer __cur_node;
        try
          {
            for (__cur_node = this->_M_impl._M_start._M_node;
                 __cur_node < this->_M_impl._M_finish._M_node;
                 ++__cur_node)
       {
  _ForwardIterator __mid = __first;
  std::advance(__mid, _S_buffer_size());
  std::__uninitialized_copy_a(__first, __mid, *__cur_node,
         _M_get_Tp_allocator());
  __first = __mid;
       }
            std::__uninitialized_copy_a(__first, __last,
     this->_M_impl._M_finish._M_first,
     _M_get_Tp_allocator());
          }
        catch(...)
          {
            std::_Destroy(this->_M_impl._M_start,
     iterator(*__cur_node, __cur_node),
     _M_get_Tp_allocator());
            throw;
          }
      }

  // Called only if _M_impl._M_finish._M_cur == _M_impl._M_finish._M_last - 1.
  template<typename _Tp, typename _Alloc>






      void
      deque<_Tp, _Alloc>::
      _M_push_back_aux(__const__ value_type& __t)

      {
 _M_reserve_map_at_back();
 *(this->_M_impl._M_finish._M_node + 1) = this->_M_allocate_node();
 try
   {




     this->_M_impl.construct(this->_M_impl._M_finish._M_cur, __t);

     this->_M_impl._M_finish._M_set_node(this->_M_impl._M_finish._M_node
      + 1);
     this->_M_impl._M_finish._M_cur = this->_M_impl._M_finish._M_first;
   }
 catch(...)
   {
     _M_deallocate_node(*(this->_M_impl._M_finish._M_node + 1));
     throw;
   }
      }

  // Called only if _M_impl._M_start._M_cur == _M_impl._M_start._M_first.
  template<typename _Tp, typename _Alloc>






      void
      deque<_Tp, _Alloc>::
      _M_push_front_aux(__const__ value_type& __t)

      {
 _M_reserve_map_at_front();
 *(this->_M_impl._M_start._M_node - 1) = this->_M_allocate_node();
 try
   {
     this->_M_impl._M_start._M_set_node(this->_M_impl._M_start._M_node
            - 1);
     this->_M_impl._M_start._M_cur = this->_M_impl._M_start._M_last - 1;




     this->_M_impl.construct(this->_M_impl._M_start._M_cur, __t);

   }
 catch(...)
   {
     ++this->_M_impl._M_start;
     _M_deallocate_node(*(this->_M_impl._M_start._M_node - 1));
     throw;
   }
      }

  // Called only if _M_impl._M_finish._M_cur == _M_impl._M_finish._M_first.
  template <typename _Tp, typename _Alloc>
    void deque<_Tp, _Alloc>::
    _M_pop_back_aux()
    {
      _M_deallocate_node(this->_M_impl._M_finish._M_first);
      this->_M_impl._M_finish._M_set_node(this->_M_impl._M_finish._M_node - 1);
      this->_M_impl._M_finish._M_cur = this->_M_impl._M_finish._M_last - 1;
      this->_M_impl.destroy(this->_M_impl._M_finish._M_cur);
    }

  // Called only if _M_impl._M_start._M_cur == _M_impl._M_start._M_last - 1.
  // Note that if the deque has at least one element (a precondition for this
  // member function), and if
  //   _M_impl._M_start._M_cur == _M_impl._M_start._M_last,
  // then the deque must have at least two nodes.
  template <typename _Tp, typename _Alloc>
    void deque<_Tp, _Alloc>::
    _M_pop_front_aux()
    {
      this->_M_impl.destroy(this->_M_impl._M_start._M_cur);
      _M_deallocate_node(this->_M_impl._M_start._M_first);
      this->_M_impl._M_start._M_set_node(this->_M_impl._M_start._M_node + 1);
      this->_M_impl._M_start._M_cur = this->_M_impl._M_start._M_first;
    }

  template <typename _Tp, typename _Alloc>
    template <typename _InputIterator>
      void
      deque<_Tp, _Alloc>::
      _M_range_insert_aux(iterator __pos,
                          _InputIterator __first, _InputIterator __last,
                          std::input_iterator_tag)
      { std::copy(__first, __last, std::inserter(*this, __pos)); }

  template <typename _Tp, typename _Alloc>
    template <typename _ForwardIterator>
      void
      deque<_Tp, _Alloc>::
      _M_range_insert_aux(iterator __pos,
                          _ForwardIterator __first, _ForwardIterator __last,
                          std::forward_iterator_tag)
      {
        __const__ size_type __n = std::distance(__first, __last);
        if (__pos._M_cur == this->_M_impl._M_start._M_cur)
   {
     iterator __new_start = _M_reserve_elements_at_front(__n);
     try
       {
  std::__uninitialized_copy_a(__first, __last, __new_start,
         _M_get_Tp_allocator());
  this->_M_impl._M_start = __new_start;
       }
     catch(...)
       {
  _M_destroy_nodes(__new_start._M_node,
     this->_M_impl._M_start._M_node);
  throw;
       }
   }
        else if (__pos._M_cur == this->_M_impl._M_finish._M_cur)
   {
     iterator __new_finish = _M_reserve_elements_at_back(__n);
     try
       {
  std::__uninitialized_copy_a(__first, __last,
         this->_M_impl._M_finish,
         _M_get_Tp_allocator());
  this->_M_impl._M_finish = __new_finish;
       }
     catch(...)
       {
  _M_destroy_nodes(this->_M_impl._M_finish._M_node + 1,
     __new_finish._M_node + 1);
  throw;
       }
   }
        else
          _M_insert_aux(__pos, __first, __last, __n);
      }

  template<typename _Tp, typename _Alloc>
# 514 "/usr/include/c++/4.5/bits/deque.tcc" 3
    typename deque<_Tp, _Alloc>::iterator
      deque<_Tp, _Alloc>::
      _M_insert_aux(iterator __pos, __const__ value_type& __x)
      {
 value_type __x_copy = __x; // XXX copy

 difference_type __index = __pos - this->_M_impl._M_start;
 if (static_cast<size_type>(__index) < size() / 2)
   {
     push_front((front()));
     iterator __front1 = this->_M_impl._M_start;
     ++__front1;
     iterator __front2 = __front1;
     ++__front2;
     __pos = this->_M_impl._M_start + __index;
     iterator __pos1 = __pos;
     ++__pos1;
     std::copy(__front2, __pos1, __front1);
   }
 else
   {
     push_back((back()));
     iterator __back1 = this->_M_impl._M_finish;
     --__back1;
     iterator __back2 = __back1;
     --__back2;
     __pos = this->_M_impl._M_start + __index;
     std::copy_backward(__pos, __back2, __back1);
   }
 *__pos = (__x_copy);
 return __pos;
      }

  template <typename _Tp, typename _Alloc>
    void
    deque<_Tp, _Alloc>::
    _M_insert_aux(iterator __pos, size_type __n, __const__ value_type& __x)
    {
      __const__ difference_type __elems_before = __pos - this->_M_impl._M_start;
      __const__ size_type __length = this->size();
      value_type __x_copy = __x;
      if (__elems_before < difference_type(__length / 2))
 {
   iterator __new_start = _M_reserve_elements_at_front(__n);
   iterator __old_start = this->_M_impl._M_start;
   __pos = this->_M_impl._M_start + __elems_before;
   try
     {
       if (__elems_before >= difference_type(__n))
  {
    iterator __start_n = (this->_M_impl._M_start
     + difference_type(__n));
    std::__uninitialized_move_a(this->_M_impl._M_start,
           __start_n, __new_start,
           _M_get_Tp_allocator());
    this->_M_impl._M_start = __new_start;
    std::copy(__start_n, __pos, __old_start);
    std::fill(__pos - difference_type(__n), __pos, __x_copy);
  }
       else
  {
    std::__uninitialized_move_fill(this->_M_impl._M_start,
       __pos, __new_start,
       this->_M_impl._M_start,
       __x_copy,
       _M_get_Tp_allocator());
    this->_M_impl._M_start = __new_start;
    std::fill(__old_start, __pos, __x_copy);
  }
     }
   catch(...)
     {
       _M_destroy_nodes(__new_start._M_node,
          this->_M_impl._M_start._M_node);
       throw;
     }
 }
      else
 {
   iterator __new_finish = _M_reserve_elements_at_back(__n);
   iterator __old_finish = this->_M_impl._M_finish;
   __const__ difference_type __elems_after =
     difference_type(__length) - __elems_before;
   __pos = this->_M_impl._M_finish - __elems_after;
   try
     {
       if (__elems_after > difference_type(__n))
  {
    iterator __finish_n = (this->_M_impl._M_finish
      - difference_type(__n));
    std::__uninitialized_move_a(__finish_n,
           this->_M_impl._M_finish,
           this->_M_impl._M_finish,
           _M_get_Tp_allocator());
    this->_M_impl._M_finish = __new_finish;
    std::copy_backward(__pos, __finish_n, __old_finish);
    std::fill(__pos, __pos + difference_type(__n), __x_copy);
  }
       else
  {
    std::__uninitialized_fill_move(this->_M_impl._M_finish,
       __pos + difference_type(__n),
       __x_copy, __pos,
       this->_M_impl._M_finish,
       _M_get_Tp_allocator());
    this->_M_impl._M_finish = __new_finish;
    std::fill(__pos, __old_finish, __x_copy);
  }
     }
   catch(...)
     {
       _M_destroy_nodes(this->_M_impl._M_finish._M_node + 1,
          __new_finish._M_node + 1);
       throw;
     }
 }
    }

  template <typename _Tp, typename _Alloc>
    template <typename _ForwardIterator>
      void
      deque<_Tp, _Alloc>::
      _M_insert_aux(iterator __pos,
                    _ForwardIterator __first, _ForwardIterator __last,
                    size_type __n)
      {
        __const__ difference_type __elemsbefore = __pos - this->_M_impl._M_start;
        __const__ size_type __length = size();
        if (static_cast<size_type>(__elemsbefore) < __length / 2)
   {
     iterator __new_start = _M_reserve_elements_at_front(__n);
     iterator __old_start = this->_M_impl._M_start;
     __pos = this->_M_impl._M_start + __elemsbefore;
     try
       {
  if (__elemsbefore >= difference_type(__n))
    {
      iterator __start_n = (this->_M_impl._M_start
       + difference_type(__n));
      std::__uninitialized_move_a(this->_M_impl._M_start,
      __start_n, __new_start,
      _M_get_Tp_allocator());
      this->_M_impl._M_start = __new_start;
      std::copy(__start_n, __pos, __old_start);
      std::copy(__first, __last, __pos - difference_type(__n));
    }
  else
    {
      _ForwardIterator __mid = __first;
      std::advance(__mid, difference_type(__n) - __elemsbefore);
      std::__uninitialized_move_copy(this->_M_impl._M_start,
         __pos, __first, __mid,
         __new_start,
         _M_get_Tp_allocator());
      this->_M_impl._M_start = __new_start;
      std::copy(__mid, __last, __old_start);
    }
       }
     catch(...)
       {
  _M_destroy_nodes(__new_start._M_node,
     this->_M_impl._M_start._M_node);
  throw;
       }
   }
        else
        {
          iterator __new_finish = _M_reserve_elements_at_back(__n);
          iterator __old_finish = this->_M_impl._M_finish;
          __const__ difference_type __elemsafter =
            difference_type(__length) - __elemsbefore;
          __pos = this->_M_impl._M_finish - __elemsafter;
          try
            {
              if (__elemsafter > difference_type(__n))
  {
    iterator __finish_n = (this->_M_impl._M_finish
      - difference_type(__n));
    std::__uninitialized_move_a(__finish_n,
           this->_M_impl._M_finish,
           this->_M_impl._M_finish,
           _M_get_Tp_allocator());
    this->_M_impl._M_finish = __new_finish;
    std::copy_backward(__pos, __finish_n, __old_finish);
    std::copy(__first, __last, __pos);
  }
              else
  {
    _ForwardIterator __mid = __first;
    std::advance(__mid, __elemsafter);
    std::__uninitialized_copy_move(__mid, __last, __pos,
       this->_M_impl._M_finish,
       this->_M_impl._M_finish,
       _M_get_Tp_allocator());
    this->_M_impl._M_finish = __new_finish;
    std::copy(__first, __mid, __pos);
  }
            }
          catch(...)
            {
              _M_destroy_nodes(this->_M_impl._M_finish._M_node + 1,
          __new_finish._M_node + 1);
              throw;
            }
        }
      }

   template<typename _Tp, typename _Alloc>
     void
     deque<_Tp, _Alloc>::
     _M_destroy_data_aux(iterator __first, iterator __last)
     {
       for (_Map_pointer __node = __first._M_node + 1;
     __node < __last._M_node; ++__node)
  std::_Destroy(*__node, *__node + _S_buffer_size(),
         _M_get_Tp_allocator());

       if (__first._M_node != __last._M_node)
  {
    std::_Destroy(__first._M_cur, __first._M_last,
    _M_get_Tp_allocator());
    std::_Destroy(__last._M_first, __last._M_cur,
    _M_get_Tp_allocator());
  }
       else
  std::_Destroy(__first._M_cur, __last._M_cur,
         _M_get_Tp_allocator());
     }

  template <typename _Tp, typename _Alloc>
    void
    deque<_Tp, _Alloc>::
    _M_new_elements_at_front(size_type __new_elems)
    {
      if (this->max_size() - this->size() < __new_elems)
 __throw_length_error(("deque::_M_new_elements_at_front"));

      __const__ size_type __new_nodes = ((__new_elems + _S_buffer_size() - 1)
         / _S_buffer_size());
      _M_reserve_map_at_front(__new_nodes);
      size_type __i;
      try
        {
          for (__i = 1; __i <= __new_nodes; ++__i)
            *(this->_M_impl._M_start._M_node - __i) = this->_M_allocate_node();
        }
      catch(...)
        {
          for (size_type __j = 1; __j < __i; ++__j)
            _M_deallocate_node(*(this->_M_impl._M_start._M_node - __j));
          throw;
        }
    }

  template <typename _Tp, typename _Alloc>
    void
    deque<_Tp, _Alloc>::
    _M_new_elements_at_back(size_type __new_elems)
    {
      if (this->max_size() - this->size() < __new_elems)
 __throw_length_error(("deque::_M_new_elements_at_back"));

      __const__ size_type __new_nodes = ((__new_elems + _S_buffer_size() - 1)
         / _S_buffer_size());
      _M_reserve_map_at_back(__new_nodes);
      size_type __i;
      try
        {
          for (__i = 1; __i <= __new_nodes; ++__i)
            *(this->_M_impl._M_finish._M_node + __i) = this->_M_allocate_node();
        }
      catch(...)
        {
          for (size_type __j = 1; __j < __i; ++__j)
            _M_deallocate_node(*(this->_M_impl._M_finish._M_node + __j));
          throw;
        }
    }

  template <typename _Tp, typename _Alloc>
    void
    deque<_Tp, _Alloc>::
    _M_reallocate_map(size_type __nodes_to_add, bool __add_at_front)
    {
      __const__ size_type __old_num_nodes
 = this->_M_impl._M_finish._M_node - this->_M_impl._M_start._M_node + 1;
      __const__ size_type __new_num_nodes = __old_num_nodes + __nodes_to_add;

      _Map_pointer __new_nstart;
      if (this->_M_impl._M_map_size > 2 * __new_num_nodes)
 {
   __new_nstart = this->_M_impl._M_map + (this->_M_impl._M_map_size
      - __new_num_nodes) / 2
                  + (__add_at_front ? __nodes_to_add : 0);
   if (__new_nstart < this->_M_impl._M_start._M_node)
     std::copy(this->_M_impl._M_start._M_node,
        this->_M_impl._M_finish._M_node + 1,
        __new_nstart);
   else
     std::copy_backward(this->_M_impl._M_start._M_node,
          this->_M_impl._M_finish._M_node + 1,
          __new_nstart + __old_num_nodes);
 }
      else
 {
   size_type __new_map_size = this->_M_impl._M_map_size
                              + std::max(this->_M_impl._M_map_size,
      __nodes_to_add) + 2;

   _Map_pointer __new_map = this->_M_allocate_map(__new_map_size);
   __new_nstart = __new_map + (__new_map_size - __new_num_nodes) / 2
                  + (__add_at_front ? __nodes_to_add : 0);
   std::copy(this->_M_impl._M_start._M_node,
      this->_M_impl._M_finish._M_node + 1,
      __new_nstart);
   _M_deallocate_map(this->_M_impl._M_map, this->_M_impl._M_map_size);

   this->_M_impl._M_map = __new_map;
   this->_M_impl._M_map_size = __new_map_size;
 }

      this->_M_impl._M_start._M_set_node(__new_nstart);
      this->_M_impl._M_finish._M_set_node(__new_nstart + __old_num_nodes - 1);
    }

  // Overload for deque::iterators, exploiting the "segmented-iterator
  // optimization".
  template<typename _Tp>
    void
    fill(__const__ _Deque_iterator<_Tp, _Tp&, _Tp*>& __first,
  __const__ _Deque_iterator<_Tp, _Tp&, _Tp*>& __last, __const__ _Tp& __value)
    {
      typedef typename _Deque_iterator<_Tp, _Tp&, _Tp*>::_Self _Self;

      for (typename _Self::_Map_pointer __node = __first._M_node + 1;
           __node < __last._M_node; ++__node)
 std::fill(*__node, *__node + _Self::_S_buffer_size(), __value);

      if (__first._M_node != __last._M_node)
 {
   std::fill(__first._M_cur, __first._M_last, __value);
   std::fill(__last._M_first, __last._M_cur, __value);
 }
      else
 std::fill(__first._M_cur, __last._M_cur, __value);
    }

  template<typename _Tp>
    _Deque_iterator<_Tp, _Tp&, _Tp*>
    copy(_Deque_iterator<_Tp, __const__ _Tp&, __const__ _Tp*> __first,
  _Deque_iterator<_Tp, __const__ _Tp&, __const__ _Tp*> __last,
  _Deque_iterator<_Tp, _Tp&, _Tp*> __result)
    {
      typedef typename _Deque_iterator<_Tp, _Tp&, _Tp*>::_Self _Self;
      typedef typename _Self::difference_type difference_type;

      difference_type __len = __last - __first;
      while (__len > 0)
 {
   __const__ difference_type __clen
     = std::min(__len, std::min(__first._M_last - __first._M_cur,
           __result._M_last - __result._M_cur));
   std::copy(__first._M_cur, __first._M_cur + __clen, __result._M_cur);
   __first += __clen;
   __result += __clen;
   __len -= __clen;
 }
      return __result;
    }

  template<typename _Tp>
    _Deque_iterator<_Tp, _Tp&, _Tp*>
    copy_backward(_Deque_iterator<_Tp, __const__ _Tp&, __const__ _Tp*> __first,
    _Deque_iterator<_Tp, __const__ _Tp&, __const__ _Tp*> __last,
    _Deque_iterator<_Tp, _Tp&, _Tp*> __result)
    {
      typedef typename _Deque_iterator<_Tp, _Tp&, _Tp*>::_Self _Self;
      typedef typename _Self::difference_type difference_type;

      difference_type __len = __last - __first;
      while (__len > 0)
 {
   difference_type __llen = __last._M_cur - __last._M_first;
   _Tp* __lend = __last._M_cur;

   difference_type __rlen = __result._M_cur - __result._M_first;
   _Tp* __rend = __result._M_cur;

   if (!__llen)
     {
       __llen = _Self::_S_buffer_size();
       __lend = *(__last._M_node - 1) + __llen;
     }
   if (!__rlen)
     {
       __rlen = _Self::_S_buffer_size();
       __rend = *(__result._M_node - 1) + __rlen;
     }

   __const__ difference_type __clen = std::min(__len,
        std::min(__llen, __rlen));
   std::copy_backward(__lend - __clen, __lend, __rend);
   __last -= __clen;
   __result -= __clen;
   __len -= __clen;
 }
      return __result;
    }
# 987 "/usr/include/c++/4.5/bits/deque.tcc" 3
}
# 69 "/usr/include/c++/4.5/deque" 2 3
# 62 "/usr/include/c++/4.5/queue" 2 3
# 1 "/usr/include/c++/4.5/vector" 1 3


# 62 "/usr/include/c++/4.5/bits/stl_queue.h" 2 3

namespace std __attribute__ ((__visibility__ ("default"))) {

  /**
   *  @brief  A standard container giving FIFO behavior.
   *
   *  @ingroup sequences
   *
   *  Meets many of the requirements of a
   *  <a href="tables.html#65">container</a>,
   *  but does not define anything to do with iterators.  Very few of the
   *  other standard container interfaces are defined.
   *
   *  This is not a true container, but an @e adaptor.  It holds another
   *  container, and provides a wrapper interface to that container.  The
   *  wrapper is what enforces strict first-in-first-out %queue behavior.
   *
   *  The second template parameter defines the type of the underlying
   *  sequence/container.  It defaults to std::deque, but it can be any type
   *  that supports @c front, @c back, @c push_back, and @c pop_front,
   *  such as std::list or an appropriate user-defined type.
   *
   *  Members not found in @a normal containers are @c container_type,
   *  which is a typedef for the second Sequence parameter, and @c push and
   *  @c pop, which are standard %queue/FIFO operations.
  */
  template<typename _Tp, typename _Sequence = deque<_Tp> >
    class queue
    {
      // concept requirements
      typedef typename _Sequence::value_type _Sequence_value_type;
     
     
     
     

      template<typename _Tp1, typename _Seq1>
        friend bool
        operator==(__const__ queue<_Tp1, _Seq1>&, __const__ queue<_Tp1, _Seq1>&);

      template<typename _Tp1, typename _Seq1>
        friend bool
        operator<(__const__ queue<_Tp1, _Seq1>&, __const__ queue<_Tp1, _Seq1>&);

    public:
      typedef typename _Sequence::value_type value_type;
      typedef typename _Sequence::reference reference;
      typedef typename _Sequence::const_reference const_reference;
      typedef typename _Sequence::size_type size_type;
      typedef _Sequence container_type;

    protected:
      /**
       *  'c' is the underlying container.  Maintainers wondering why
       *  this isn't uglified as per style guidelines should note that
       *  this name is specified in the standard, [23.2.3.1].  (Why?
       *  Presumably for the same reason that it's protected instead
       *  of private: to allow derivation.  But none of the other
       *  containers allow for derivation.  Odd.)
       */
      _Sequence c;

    public:
      /**
       *  @brief  Default constructor creates no elements.
       */

      explicit
      queue(__const__ _Sequence& __c = _Sequence())
      : c(__c) { }
# 152 "/usr/include/c++/4.5/bits/stl_queue.h" 3
      /**
       *  Returns true if the %queue is empty.
       */
      bool
      empty() __const__
      { return c.empty(); }

      /**  Returns the number of elements in the %queue.  */
      size_type
      size() __const__
      { return c.size(); }

      /**
       *  Returns a read/write reference to the data at the first
       *  element of the %queue.
       */
      reference
      front()
      {
 ;
 return c.front();
      }

      /**
       *  Returns a read-only (constant) reference to the data at the first
       *  element of the %queue.
       */
      const_reference
      front() __const__
      {
 ;
 return c.front();
      }

      /**
       *  Returns a read/write reference to the data at the last
       *  element of the %queue.
       */
      reference
      back()
      {
 ;
 return c.back();
      }

      /**
       *  Returns a read-only (constant) reference to the data at the last
       *  element of the %queue.
       */
      const_reference
      back() __const__
      {
 ;
 return c.back();
      }

      /**
       *  @brief  Add data to the end of the %queue.
       *  @param  x  Data to be added.
       *
       *  This is a typical %queue operation.  The function creates an
       *  element at the end of the %queue and assigns the given data
       *  to it.  The time complexity of the operation depends on the
       *  underlying sequence.
       */
      void
      push(__const__ value_type& __x)
      { c.push_back(__x); }
# 232 "/usr/include/c++/4.5/bits/stl_queue.h" 3
      /**
       *  @brief  Removes first element.
       *
       *  This is a typical %queue operation.  It shrinks the %queue by one.
       *  The time complexity of the operation depends on the underlying
       *  sequence.
       *
       *  Note that no data is returned, and if the first element's
       *  data is needed, it should be retrieved before pop() is
       *  called.
       */
      void
      pop()
      {
 ;
 c.pop_front();
      }






    };

  /**
   *  @brief  Queue equality comparison.
   *  @param  x  A %queue.
   *  @param  y  A %queue of the same type as @a x.
   *  @return  True iff the size and elements of the queues are equal.
   *
   *  This is an equivalence relation.  Complexity and semantics depend on the
   *  underlying sequence type, but the expected rules are:  this relation is
   *  linear in the size of the sequences, and queues are considered equivalent
   *  if their sequences compare equal.
  */
  template<typename _Tp, typename _Seq>
    __inline__ bool
    operator==(__const__ queue<_Tp, _Seq>& __x, __const__ queue<_Tp, _Seq>& __y)
    { return __x.c == __y.c; }

  /**
   *  @brief  Queue ordering relation.
   *  @param  x  A %queue.
   *  @param  y  A %queue of the same type as @a x.
   *  @return  True iff @a x is lexicographically less than @a y.
   *
   *  This is an total ordering relation.  Complexity and semantics
   *  depend on the underlying sequence type, but the expected rules
   *  are: this relation is linear in the size of the sequences, the
   *  elements must be comparable with @c <, and
   *  std::lexicographical_compare() is usually used to make the
   *  determination.
  */
  template<typename _Tp, typename _Seq>
    __inline__ bool
    operator<(__const__ queue<_Tp, _Seq>& __x, __const__ queue<_Tp, _Seq>& __y)
    { return __x.c < __y.c; }

  /// Based on operator==
  template<typename _Tp, typename _Seq>
    __inline__ bool
    operator!=(__const__ queue<_Tp, _Seq>& __x, __const__ queue<_Tp, _Seq>& __y)
    { return !(__x == __y); }

  /// Based on operator<
  template<typename _Tp, typename _Seq>
    __inline__ bool
    operator>(__const__ queue<_Tp, _Seq>& __x, __const__ queue<_Tp, _Seq>& __y)
    { return __y < __x; }

  /// Based on operator<
  template<typename _Tp, typename _Seq>
    __inline__ bool
    operator<=(__const__ queue<_Tp, _Seq>& __x, __const__ queue<_Tp, _Seq>& __y)
    { return !(__y < __x); }

  /// Based on operator<
  template<typename _Tp, typename _Seq>
    __inline__ bool
    operator>=(__const__ queue<_Tp, _Seq>& __x, __const__ queue<_Tp, _Seq>& __y)
    { return !(__x < __y); }
# 322 "/usr/include/c++/4.5/bits/stl_queue.h" 3
  /**
   *  @brief  A standard container automatically sorting its contents.
   *
   *  @ingroup sequences
   *
   *  This is not a true container, but an @e adaptor.  It holds
   *  another container, and provides a wrapper interface to that
   *  container.  The wrapper is what enforces priority-based sorting 
   *  and %queue behavior.  Very few of the standard container/sequence
   *  interface requirements are met (e.g., iterators).
   *
   *  The second template parameter defines the type of the underlying
   *  sequence/container.  It defaults to std::vector, but it can be
   *  any type that supports @c front(), @c push_back, @c pop_back,
   *  and random-access iterators, such as std::deque or an
   *  appropriate user-defined type.
   *
   *  The third template parameter supplies the means of making
   *  priority comparisons.  It defaults to @c less<value_type> but
   *  can be anything defining a strict weak ordering.
   *
   *  Members not found in @a normal containers are @c container_type,
   *  which is a typedef for the second Sequence parameter, and @c
   *  push, @c pop, and @c top, which are standard %queue operations.
   *
   *  @note No equality/comparison operators are provided for
   *  %priority_queue.
   *
   *  @note Sorting of the elements takes place as they are added to,
   *  and removed from, the %priority_queue using the
   *  %priority_queue's member functions.  If you access the elements
   *  by other means, and change their data such that the sorting
   *  order would be different, the %priority_queue will not re-sort
   *  the elements for you.  (How could it know to do so?)
  */
  template<typename _Tp, typename _Sequence = vector<_Tp>,
    typename _Compare = less<typename _Sequence::value_type> >
    class priority_queue
    {
      // concept requirements
      typedef typename _Sequence::value_type _Sequence_value_type;
     
     
     
     
     


    public:
      typedef typename _Sequence::value_type value_type;
      typedef typename _Sequence::reference reference;
      typedef typename _Sequence::const_reference const_reference;
      typedef typename _Sequence::size_type size_type;
      typedef _Sequence container_type;

    protected:
      //  See queue::c for notes on these names.
      _Sequence c;
      _Compare comp;

    public:
      /**
       *  @brief  Default constructor creates no elements.
       */

      explicit
      priority_queue(__const__ _Compare& __x = _Compare(),
       __const__ _Sequence& __s = _Sequence())
      : c(__s), comp(__x)
      { std::make_heap(c.begin(), c.end(), comp); }
# 406 "/usr/include/c++/4.5/bits/stl_queue.h" 3
      /**
       *  @brief  Builds a %queue from a range.
       *  @param  first  An input iterator.
       *  @param  last  An input iterator.
       *  @param  x  A comparison functor describing a strict weak ordering.
       *  @param  s  An initial sequence with which to start.
       *
       *  Begins by copying @a s, inserting a copy of the elements
       *  from @a [first,last) into the copy of @a s, then ordering
       *  the copy according to @a x.
       *
       *  For more information on function objects, see the
       *  documentation on @link functors functor base
       *  classes@endlink.
       */

      template<typename _InputIterator>
        priority_queue(_InputIterator __first, _InputIterator __last,
         __const__ _Compare& __x = _Compare(),
         __const__ _Sequence& __s = _Sequence())
 : c(__s), comp(__x)
        {
   ;
   c.insert(c.end(), __first, __last);
   std::make_heap(c.begin(), c.end(), comp);
 }
# 467 "/usr/include/c++/4.5/bits/stl_queue.h" 3
      /**
       *  Returns true if the %queue is empty.
       */
      bool
      empty() __const__
      { return c.empty(); }

      /**  Returns the number of elements in the %queue.  */
      size_type
      size() __const__
      { return c.size(); }

      /**
       *  Returns a read-only (constant) reference to the data at the first
       *  element of the %queue.
       */
      const_reference
      top() __const__
      {
 ;
 return c.front();
      }

      /**
       *  @brief  Add data to the %queue.
       *  @param  x  Data to be added.
       *
       *  This is a typical %queue operation.
       *  The time complexity of the operation depends on the underlying
       *  sequence.
       */
      void
      push(__const__ value_type& __x)
      {
 c.push_back(__x);
 std::push_heap(c.begin(), c.end(), comp);
      }
# 522 "/usr/include/c++/4.5/bits/stl_queue.h" 3
      /**
       *  @brief  Removes first element.
       *
       *  This is a typical %queue operation.  It shrinks the %queue
       *  by one.  The time complexity of the operation depends on the
       *  underlying sequence.
       *
       *  Note that no data is returned, and if the first element's
       *  data is needed, it should be retrieved before pop() is
       *  called.
       */
      void
      pop()
      {
 ;
 std::pop_heap(c.begin(), c.end(), comp);
 c.pop_back();
      }
# 550 "/usr/include/c++/4.5/bits/stl_queue.h" 3
    };

  // No equality/comparison operators are provided for priority_queue.
# 562 "/usr/include/c++/4.5/bits/stl_queue.h" 3
}
# 66 "/usr/include/c++/4.5/queue" 2 3
# 26 "/usr/local/include/G3D/TextInput.h" 2 3
# 1 "/usr/include/ctype.h" 1 3 4
/* Copyright (C) 1991,92,93,95,96,97,98,99,2001,2002,2004,2007,2008,2009
   	Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

/*
 *	ISO C99 Standard 7.4: Character handling	<ctype.h>
 */
# 27 "/usr/local/include/G3D/TextInput.h" 2 3
# 1 "/usr/include/stdio.h" 1 3 4
/* Define ISO C stdio on top of C++ iostreams.
   Copyright (C) 1991, 1994-2008, 2009, 2010 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

/*
 *	ISO C99 Standard: 7.19 Input/output	<stdio.h>
 */


# 28 "/usr/local/include/G3D/TextInput.h" 2 3

namespace G3D {

/**
 For use with TextInput.
 */
class Token {
public:

    /**
     More detailed type information than Type.
     */
    enum ExtendedType {
      DOUBLE_QUOTED_TYPE,
      SINGLE_QUOTED_TYPE,
      SYMBOL_TYPE,
      FLOATING_POINT_TYPE,
      INTEGER_TYPE,
      BOOLEAN_TYPE,
      LINE_COMMENT_TYPE,
      BLOCK_COMMENT_TYPE,
      NEWLINE_TYPE,
      END_TYPE
    };

    /**
     Strings are enclosed in quotes, symbols are not.
     */
    enum Type {
      STRING = DOUBLE_QUOTED_TYPE,
      SYMBOL = SYMBOL_TYPE,
      NUMBER = FLOATING_POINT_TYPE,
      BOOLEAN = BOOLEAN_TYPE,
      COMMENT = LINE_COMMENT_TYPE,
      NEWLINE = NEWLINE_TYPE,
      END = END_TYPE
    };

private:

    friend class TextInput;

    /** 
      Holds the actual value, which might be any type.  If a number, it will be 
      parsed at runtime.
    */
    std::string _string;

    bool _bool;
    int _line;
    int _character;
    uint64 _bytePosition;
    Type _type;
    ExtendedType _extendedType;

public:

    Token() :
        _string(""),
        _bool(false),
        _line(0),
        _character(0),
        _bytePosition(0),
        _type(END),
        _extendedType(END_TYPE) {}

    Token(Type t, ExtendedType e, __const__ std::string& s, int L, int c, uint64 byte)
        : _string(s), _bool(false), _line(L), _character(c), _bytePosition(byte), _type(t), _extendedType(e) {}

    Token(Type t, ExtendedType e, __const__ std::string& s, bool b, int L, int c, uint64 byte)
        : _string(s), _bool(b), _line(L), _character(c), _bytePosition(byte), _type(t), _extendedType(e) {}

    Type type() __const__ {
        return _type;
    }

    ExtendedType extendedType() __const__ {
        return _extendedType;
    }

    /**
     The value of a single or double quote string (not including the quotes),
     the name of a symbol, or the exact textual representation of a number as
     parsed from the input. 
     */
    __const__ std::string& string() __const__ {
        return _string;
    }

    bool boolean() __const__ {
        return _bool;
    }

    /**
     Starting line of the input from which this token was parsed.  Starts
     at 1.
     */
    int line() __const__ {
        return _line;
    }

    /**
     Starting character position in the input line from which this token was
     parsed.  Starts at 1.
     */
    int character() __const__ {
        return _character;
    }

    /** Number of bytes from the beginning of the buffer that this token was parsed from. 
      Begins at 0 */
    uint64 bytePosition() __const__ {
        return _bytePosition;
    }

    /** Return the numeric value for a number type, or zero if this is
        not a number type.
    */
    double number() __const__;
};


/**
 A simple style tokenizer for reading text files.  TextInput handles a
 superset of C++,Java, Matlab, and Bash code text including single
 line comments, block comments, quoted strings with escape sequences,
 and operators.  TextInput recognizes several categories of tokens,
 which are separated by white space, quotation marks, or the end of a
 recognized operator:

 <ul>
  <li><CODE>Token::SINGLE_QUOTED_TYPE</CODE> string of characters surrounded by single quotes, e.g., 'x', '\\0', 'foo'.
  <li><CODE>Token::DOUBLE_QUOTED_TYPE</CODE> string of characters surrounded by double quotes, e.g., "x", "abc\txyz", "b o b".
  <li><CODE>Token::SYMBOL_TYPE</CODE> legal C++ operators, keywords, and identifiers.  e.g., >=, Foo, _X, class, {
  <li><CODE>Token::INTEGER_TYPE</CODE> numbers without decimal places or exponential notation. e.g., 10, 0x17F, 32, 0, -155
  <li><CODE>Token::FLOATING_POINT_TYPE</CODE> numbers with decimal places or exponential notation. e.g., 1e3, -1.2, .4, 0.5
  <li><CODE>Token::BOOLEAN_TYPE</CODE> special symbols like "true" and "false"; the exact details can be configured in TextInput::Settings
  <li><CODE>Token::LINE_COMMENT_TYPE</CODE> (disabled by default); generated for line comments as specified by TextInput::Settings
  <li><CODE>Token::BLOCK_COMMENT_TYPE</CODE> (disabled by default); generated for c-style block comments as specified by TextInput::Settings
  <li><CODE>Token::NEWLINE_TYPE</CODE> (disabled by default); generated for any of "\\r", "\\n" or "\\r\\n"
 </ul>

 <P>The special ".." and "..." tokens are always recognized in
 addition to normal C++ operators. Additional tokens can be made
 available by changing the Settings.

 Negative numbers are handled specially because of the ambiguity between unary minus and negative numbers-- 
 see the note on TextInput::read.

  TextInput does not have helper functions for types with non-obvious
  formatting, or helpers that would be redundant.  Use the serialize
  methods instead for parsing specific types like int, Vector3, and
  Color3.

  Inside quoted strings escape sequences are converted.  Thus the
  string token for ["a\\nb"] is 'a', followed by a newline, followed by
  'b'.  Outside of quoted strings, escape sequences are not converted,
  so the token sequence for [a\\nb] is symbol 'a', symbol '\\', symbol
  'nb' (this matches what a C++ parser would do).  The exception is
  that a specified TextInput::Settings::otherCommentCharacter preceeded
  by a backslash is assumed to be an escaped comment character and is
  returned as a symbol token instead of being parsed as a comment
  (this is what a LaTex or VRML parser would do).

  <B>Examples</B>

  <PRE>
  TextInput ti(TextInput::FROM_STRING, "name = \"Max\", height = 6");

  Token t;

  t = ti.read(); 
  debugAssert(t.type == Token::SYMBOL);
  debugAssert(t.sval == "name");

  ti.read();
  debugAssert(t.type == Token::SYMBOL);
  debugAssert(t.sval == "=");

  std::string name = ti.read().sval;
  ti.read();
  </PRE>

  <PRE>
  TextInput ti(TextInput::FROM_STRING, "name = \"Max\", height = 6");
  ti.readSymbols("name", "=");
  std::string name = ti.readString();
  ti.readSymbols(",", "height", "=");
  double height = ti. readNumber();
  </PRE>

 Assumes that the file is not modified once opened.
 */
class TextInput {
public:
    /** Includes MSVC specials parsing */
    static double parseNumber(__const__ std::string& _string);

    /** toLower(_string) == "true" */
    static bool parseBoolean(__const__ std::string& _string);


    /** Tokenizer configuration options.  */
    class Settings {
    public:
        /** If true, C-style slash-star marks a multi-line comment.

            See generateCommentTokens for rules on how this is applied.

            Default is true.
         */
        bool cppBlockComments;

        /** If true, // begins a single line comment.

            See generateCommentTokens for rules on how this is applied.

            Default is true.
         */
        bool cppLineComments;

        /** If true, otherCommentCharacter and otherCommentCharacter2
            are used to begin single line comments in the same way
            cppLineComments is.

            See generateCommentTokens for rules on how this is applied.

            Default is true.
         */
        bool otherLineComments;

        /** If true, \\r, \\n, \\t, \\0, \\\\ and other escape sequences inside
            strings are converted to the equivalent C++ escaped character.
            If false, backslashes are treated literally.  It is convenient to
            set to false if reading Windows paths, for example, like
            c:\\foo\\bar.

            Default is true.
         */
        bool escapeSequencesInStrings;

        /** If not '\\0', specifies a character that begins single line
            comments ('#' and '%' are popular choices).  This is independent
            of the cppLineComments flag.  If the character appears in text with
            a backslash in front of it, it is considered escaped and is not
            treated as a comment character.

            Default is '\\0'.
         */
        char otherCommentCharacter;

        /** Another (optional) 1-comment character.  Useful for files that
            support multiple comment syntaxes.  Default is '\\0'.
         */
        char otherCommentCharacter2;

        /** If true, comments enabled by cppBlockComments, cppLineComments
            and otherLineComments will generate their respective tokens.
            If false, the same settings will enable parsing and ignoring
            comments

            Default is false.
         */
        bool generateCommentTokens;

        /** If true, newlines will generate  tokens.
            If false, newlines will be discarded as whitespace when parsed
            outside of other tokens.

            Default is false.
         */
        bool generateNewlineTokens;

        /** If true, "-1" parses as the number -1 instead of the
            symbol "-" followed by the number 1.  Default is true.*/
        bool signedNumbers;

        /** If true, strings can be marked with single quotes (e.g.,
            'aaa').  If false, the quote character is parsed as a
            symbol. Default is true.  Backquote (`) is always parsed
            as a symbol. */
        bool singleQuotedStrings;

        /** The character to use as a single quote.  Defaults to "'" (backquote),
            occasionally useful to set to "`" (forward quote) or to "," (comma) for
            reading CSV files. */
        char singleQuoteCharacter;

        /** If set to a non-empty string, that string will be used in
            place of the real file name (or in place of a pseudonym
            constructed from the buffer if given FROM_STRING) in
            tokens and exceptions.
            
            Default is empty.
        */
        std::string sourceFileName;


        /** Added to the line number reported by peekLineNumber and in
            exceptions.  Useful for concatenating files that are
            parsed separately.  Default is zero. */
        int startingLineNumberOffset;

        /** 
          Parse -1.#IND00 as the floating point number returned by
          nan(), -1.#INF00 as -G3D::inf(), and 1.#INF00 as G3D::inf().  
          
          Note that the C99 standard specifies that a variety of formats
          like "nan" are to be used; these are supported by 
          G3D::TextInput::Settings::simpleFloatSpecials.

          An alternative to specifying msvcFloatSpecials is to read numbers as:
          <pre>
            Token x = t.read();
            Token y = t.peek();
            if ((x.string() == "-1.") && 
                (y.string() == "#INF00") && 
                (y.character() == x.character() + 3) &&
                (y.line() == x.line()) {
                t.read();
                return nan();
            }
            // ... similar cases for inf
          </pre>

          If the single-comment character was #, the floating point
          special format overrides the comment and will be parsed
          instead.

          If signedNumbers is false msvcFloatSpecials will not be parsed.

          Default is true. */
        bool msvcFloatSpecials;

        /** Parses "+inf', "-inf", "inf", "nan" as floats instead of symbols. 
            Defaults to true.*/
        bool simpleFloatSpecials;

        /**
         Parse the following set of useful proof symbols:
         
           =>
           ::>
           <::
           :>
           <:
           |-
           ::=
           :=
           <-

           Default is false.
        */
        bool proofSymbols;

        /**
         When parsing booleans and msvcFloatSpecials, is case significant?
         Default is {true}
        */
        bool caseSensitive;

        /** All symbols that will become the 'true' boolean token.  See also caseSensitive.
            Clear this value to disable parsing of true booleans.

            Default is {true}.
         */
        Set<std::string> trueSymbols;

        /** See trueSymbols. Default is {false}*/
        Set<std::string> falseSymbols;

        Settings();
    };

private:

    std::deque<Token> stack;

    /**
     Characters to be tokenized.
     */
    Array<char> buffer;

    /**
     Offset of current character (the next character to consumed) in
     input buffer.
     */
    int currentCharOffset;

    /**
     Line number of next character to be consumed from the input buffer.  (1
     indicates first line of input.)

     Note that this is the line number of the @e next character to be
     consumed from the input, not the line number of the @e last character
     consumed!
     */
    int lineNumber;

    /**
     Character number (within the line) of the next character to be consumed
     from the input buffer.  (1 indicates first character of the line).

     Note that this is the character number of the @e next character to be
     consumed from the input, not the character number of the @e last
     character consumed!
     */
    int charNumber;

    /** Configuration options.  This includes the file name that will be
        reported in tokens and exceptions.  */
    Settings options;

    void init();

    /**
     Consumes the next character from the input buffer, and returns that
     character.  Updates lineNumber and charNumber to reflect the location of
     the next character in the input buffer.

     Note: you shouldn't be using the return value of this function in most
     cases.  In general, you should peekInputChar() to get the next
     character, determine what to do with it, then consume it with this
     function (or with eatAndPeekInputChar()).  Given that usage, in most
     instances you already know what this function would return!
     */
    int eatInputChar();

    /**
     Returns the next character from the input buffer, without consuming any
     characters.  Can also be used to look deeper into the input buffer.
     Does not modify lineNumber or charNumber.

     @param distance Index of the character in the input buffer to peek at,
     relative to the next character.  Default is 0, for the next character in
     the input buffer.
     */
    int peekInputChar(int distance = 0);

    /**
     Helper function to consume the next character in the input buffer and
     peek at the one following (without consuming it).
     */
    __inline__ int eatAndPeekInputChar() {
        eatInputChar();
        return peekInputChar(0);
    }

    /**
     Read the next token, returning an END token if no more input is
     available.
     */
    Token nextToken();

    /**
       Helper for nextToken.  Appends characters to t._string until the end
       delimiter is reached.
       
       When called, the next character in the input buffer should be first the
       first character after the opening delimiter character.
    */
    void parseQuotedString(unsigned char delimiter, Token& t);

public:

    class TokenException : public ParseError {
    public:
        /** Name of file being parsed when exception occurred. 
            \deprecated  Use filename
         */
        std::string sourceFile;

        virtual ~TokenException() {}

    protected:

        TokenException(
            __const__ std::string& src,
            int ln,
            int ch);

    };

    /** While parsing a number of the form 1.\#IN?00, ? was 
        not 'D' or 'F'. */
    class BadMSVCSpecial : public TokenException {
    public:

        BadMSVCSpecial(
            __const__ std::string& src,
            int ln,
            int ch);
    };

    /** Thrown by the read methods. */
    class WrongTokenType : public TokenException {
    public:
        Token::Type expected;
        Token::Type actual;

        WrongTokenType(
            __const__ std::string& src,
            int ln,
            int ch,
            Token::Type e,
            Token::Type a);
    };

    class WrongSymbol : public TokenException {
    public:
        std::string expected;
        std::string actual;

        WrongSymbol(
            __const__ std::string& src,
            int ln,
            int ch,
            __const__ std::string& e,
            __const__ std::string& a);
    };


    /** String read from input did not match expected string.  */
    class WrongString : public TokenException {
    public:
        std::string expected;
        std::string actual;

        WrongString(
            __const__ std::string& src,
            int ln,
            int ch,
            __const__ std::string& e,
            __const__ std::string& a);
    };

    TextInput(__const__ std::string& filename, __const__ Settings& settings = Settings());

    enum FS {FROM_STRING};
    /** Creates input directly from a string.  The first argument must be
        TextInput::FROM_STRING.
    */
    TextInput(FS fs, __const__ std::string& str, __const__ Settings& settings = Settings());

    /** Returns true while there are tokens remaining. */
    bool hasMore();

    /** Read the next token (which will be the END token if ! hasMore()).
    
        Signed numbers can be handled in one of two modes.  If the option 
        TextInput::Settings::signedNumbers is true,
        A '+' or '-' immediately before a number is prepended onto that number and
        if there is intervening whitespace, it is read as a separate symbol.

        If TextInput::Settings::signedNumbers is false,
        read() does not distinguish between a plus or minus symbol next
        to a number and a positive/negative number itself.  For example, "x - 1" and "x -1"
        will be parsed the same way by read().  
        
        In both cases, readNumber() will contract a leading "-" or "+" onto
        a number.
    */
    Token read();

    /** Calls read() until the result is not a newline or comment */
    Token readSignificant();

    /** Read one token (or possibly two) as a number or throws
        WrongTokenType, and returns the number.

        If the first token in the input is a number, it is returned directly.

        If TextInput::Settings::signedNumbers is false and the input stream
        contains a '+' or '-' symbol token immediately followed by a number
        token, both tokens will be consumed and a single token will be
        returned by this method.

        WrongTokenType will be thrown if one of the input conditions
        described above is not satisfied.  When an exception is thrown, no
        tokens are consumed.
    */
    double readNumber();

    bool readBoolean();

    /** Reads a string token or throws WrongTokenType, and returns the token.

        Use this method (rather than readString) if you want the token's
        location as well as its value.

        WrongTokenType will be thrown if the next token in the input stream
        is not a string.  When an exception is thrown, no tokens are
        consumed.
    */
    Token readStringToken();

    /** Like readStringToken, but returns the token's string.

        Use this method (rather than readStringToken) if you want the token's
        value but don't really care about its location in the input.  Use of
        readStringToken is encouraged for better error reporting.
    */
    std::string readString();

    /** Reads a specific string token or throws either WrongTokenType or
        WrongString.  If the next token in the input is a string matching @p
        s, it will be consumed.

        Use this method if you want to match a specific string from the
        input.  In that case, typically error reporting related to the token
        is only going to occur because of a mismatch, so no location
        information is needed by the caller.

        WrongTokenType will be thrown if the next token in the input stream
        is not a string.  WrongString will be thrown if the next token in the
        input stream is a string but does not match the @p s parameter.  When
        an exception is thrown, no tokens are consumed.

        \sa readString(), readStringToken(), readUntilNewlineAsString()
      */
    void readString(__const__ std::string& s);

    /** Read from the beginning of the next token until the following newline 
      and return the result as a string, ignoring all parsing in between. The newline 
      is not returned in the string, and the following token read will be a newline or
      end of file token (if they are enabled for parsing).*/
    std::string readUntilNewlineAsString();

    /** Reads a comment token or throws WrongTokenType, and returns the token.

        Use this method (rather than readComment) if you want the token's
        location as well as its value.

        WrongTokenType will be thrown if the next token in the input stream
        is not a comment.  When an exception is thrown, no tokens are
        consumed.
    */
    Token readCommentToken();

    /** Like readCommentToken, but returns the token's string.

        Use this method (rather than readCommentToken) if you want the token's
        value but don't really care about its location in the input.  Use of
        readCommentToken is encouraged for better error reporting.
    */
    std::string readComment();

    /** Reads a specific comment token or throws either WrongTokenType or
        WrongString.  If the next token in the input is a comment matching @p
        s, it will be consumed.

        Use this method if you want to match a specific comment from the
        input.  In that case, typically error reporting related to the token
        is only going to occur because of a mismatch, so no location
        information is needed by the caller.

        WrongTokenType will be thrown if the next token in the input stream
        is not a comment.  WrongString will be thrown if the next token in the
        input stream is a comment but does not match the @p s parameter.  When
        an exception is thrown, no tokens are consumed.
      */
     void readComment(__const__ std::string& s);

    /** Reads a newline token or throws WrongTokenType, and returns the token.

        Use this method (rather than readNewline) if you want the token's
        location as well as its value.

        WrongTokenType will be thrown if the next token in the input stream
        is not a newline.  When an exception is thrown, no tokens are
        consumed.
    */
    Token readNewlineToken();

    /** Like readNewlineToken, but returns the token's string.

        Use this method (rather than readNewlineToken) if you want the token's
        value but don't really care about its location in the input.  Use of
        readNewlineToken is encouraged for better error reporting.
    */
    std::string readNewline();

    /** Reads a specific newline token or throws either WrongTokenType or
        WrongString.  If the next token in the input is a newline matching @p
        s, it will be consumed.

        Use this method if you want to match a specific newline from the
        input.  In that case, typically error reporting related to the token
        is only going to occur because of a mismatch, so no location
        information is needed by the caller.

        WrongTokenType will be thrown if the next token in the input stream
        is not a newline.  WrongString will be thrown if the next token in the
        input stream is a newlin but does not match the @p s parameter.  When
        an exception is thrown, no tokens are consumed.
      */
     void readNewline(__const__ std::string& s);

    /** Reads a symbol token or throws WrongTokenType, and returns the token.

        Use this method (rather than readSymbol) if you want the token's
        location as well as its value.

        WrongTokenType will be thrown if the next token in the input stream
        is not a symbol.  When an exception is thrown, no tokens are
        consumed.
    */
    Token readSymbolToken();

    /** Like readSymbolToken, but returns the token's string.

        Use this method (rather than readSymbolToken) if you want the token's
        value but don't really care about its location in the input.  Use of
        readSymbolToken is encouraged for better error reporting.
    */
    std::string readSymbol();

    /** Reads a specific symbol token or throws either WrongTokenType or
        WrongSymbol.  If the next token in the input is a symbol matching @p
        symbol, it will be consumed.

        Use this method if you want to match a specific symbol from the
        input.  In that case, typically error reporting related to the token
        is only going to occur because of a mismatch, so no location
        information is needed by the caller.

        WrongTokenType will be thrown if the next token in the input stream
        is not a symbol.  WrongSymbol will be thrown if the next token in the
        input stream is a symbol but does not match the @p symbol parameter.
        When an exception is thrown, no tokens are consumed.
    */
    void readSymbol(__const__ std::string& symbol);


    /** Read a series of two specific symbols.  See readSymbol.  */
    void readSymbols(__const__ std::string& s1, __const__ std::string& s2) {
        readSymbol(s1);
        readSymbol(s2);
    }

    /** Read a series of three specific symbols.  See readSymbol.  */
    void readSymbols(
        __const__ std::string& s1,
        __const__ std::string& s2,
        __const__ std::string& s3) {
        readSymbol(s1);
        readSymbol(s2);
        readSymbol(s3);
    }

    /** Read a series of four specific symbols.  See readSymbol.  */
    void readSymbols(
        __const__ std::string& s1,
        __const__ std::string& s2,
        __const__ std::string& s3,
        __const__ std::string& s4) {
        readSymbol(s1);
        readSymbol(s2);
        readSymbol(s3);
        readSymbol(s4);
    }

    /** Return a copy of the next token in the input stream, but don't remove
        it from the input stream.
    */
    Token peek();

    /** Returns the line number for the @e next token.  See also peek.  */
    int peekLineNumber();

    /** Returns the character number (relative to the line) for the @e next
        token in the input stream.  See also peek.
    */
    int peekCharacterNumber();

    /** Take a previously read token and push it back at the front of the
        input stream.

        Can be used in the case where more than one token of read-ahead is
        needed (i.e., when peek doesn't suffice).
    */
    void push(__const__ Token& t);

    /** Returns the filename from which this input is drawn, or the first few
        characters of the string if created from a string.
        If settings::filename is non-empty that will replace the
        true filename.*/
    __const__ std::string& filename() __const__;
};

void deserialize(bool& b, TextInput& ti);
void deserialize(int& b, TextInput& ti);
void deserialize(uint8& b, TextInput& ti);
void deserialize(double& b, TextInput& ti);
void deserialize(float& b, TextInput& ti);
void deserialize(std::string& b, TextInput& ti);

} // namespace


# 27 "/usr/local/include/G3D/Any.h" 2 3





namespace G3D {

class TextOutput;

/** 
\brief Easy loading and saving of human-readable configuration files.

Any encodes typed, structured data and can serialize it to a human
readable format that is very similar to the Python language's data
syntax.  It is well-suited for quickly creating human-readable file
formats, especially since deserialization and serialization preserve
comments and an Any can tell you what file and line it came from.  The
syntax allows most C++ editors to properly highlight Any files, and
makes it easy to design little ad-hoc C-like languages in
configuration files.

The class is designed so that copying Anys generally is fast, even if
it is a large array or table.  This is because data is shared between
copies until it is mutated, at which point an actual copy occurs.

\section Example
Sample File:
<pre>
{
   shape = "round",

   // in meters
   radius = 3.7,

   position = Vector3(1.0, -1.0, 0.0),
   video = { format = "RGB8", size = (320, 200)},

   material = #include("rocks.mat")
}
</pre>

Sample code using Any:
<pre>
Any x;
x.load("ball.txt");
if (x["shape"].string() == "round") {
    x["density"] = 3;
}
x.save("ball.txt");
</pre>

The custom serialization format was chosen to be terse, easy for
humans to read, and easy for machines to parse. It was specifically
chosen over formats like XML, YAML, JSON, S-expressions, and Protocol
Buffers, although there is no reason you could not write readers and
writers for G3D::Any that support those.

G3D::Any assumes that structures do not contain cycles; it is an 
error to create a structure like:

<pre>
Any x(Any::ARRAY);
x.array().append(x);    // don't do this!
</pre>

although no exception will be thrown at runtime during that append.


\section Parsing

The primary use of Any is to create your own text file formats.
The Vector3 constructor is a good example of how to use the Any::verify 
methods to provide good error checking while parsing such formats:

<pre>
Vector3::Vector3(const Any& any) {
    any.verifyName("Vector3");
    any.verifyType(Any::TABLE, Any::ARRAY);
    any.verifySize(3);

    if (any.type() == Any::ARRAY) {
        x = any[0];
        y = any[1];
        z = any[2];
    } else {
        // Table
        x = any["x"];
        y = any["y"];
        z = any["z"];
    }
}
</pre>

It is often convenient to iterate through the table portion:

<pre>
    for (Any::AnyTable::Iterator it = any.table().begin(); it.hasMore(); ++it) {
        const std::string& k = toLower(it->key);
        if (key == "hello") {
           ...
        } else if (key == "goodbye") {
           ...
        } else {
           any.verify(false, "Unsupported key: " + it->key);
        }
    }
</pre>

\section BNF
Serialized format BNF:

<pre>
identifier  ::= (letter | "_") (letter | digit | "_")*
identifier-op ::= "::" | "->" | "."

identifier-exp ::= [identifier-op] identifier (identifier-op identifier)*

comment     ::= C++ single or multi-line comments
separator   ::= "," | ";"

number      ::= <legal C printf number format>
string      ::= <legal C double-quoted string; backslashes must be escaped>
boolean     ::= "True" | "False"
none        ::= "None"
array       ::= ("(" | "[") [ value (separator value)* [separator] ] (")" | "]")
pair        ::= (identifier | string) "=" value
table       ::= "{" [ pair (separator pair)* [separator] ] "}"
named-array ::= identifier-exp array
named-table ::= identifier-exp table
include     ::= "#" "include" "(" string ")"

value       ::= [comment] (none | number | boolean | string | array | table | named-array | named-table | include)
</pre>

Except for single-line comments, whitespace is not significant.  
All parsing is case-insensitive.

The include expression pastes the contents of the named file in as if
they appeared in the original source.  Note that an include expression
can only appear in the locations where a value is expected.  This means
that it cannot yield more than one element of an array and cannot serve
as the pair in a table.

The deserializer allows the substitution of [] for () when writing
tuples and ";" for ",".  These are convenient when mimicing a
programming language, e.g., <code>"[ printf("hello world."); clearScreen();]"</code> 
parses as an array containing two named arrays within it. The 
deserializer also allows a trailing comma inside any array or table,
which also convenient when commenting out the last element.

The serializer indents four spaces for each level of nesting. 
Tables are written with the keys in alphabetic order.
*/
class Any {
public:

    enum Type {NONE, BOOLEAN, NUMBER, STRING, ARRAY, TABLE};

    static std::string toString(Type t);

    /** Where an Any came from in a file.  Useful for throwing parsing errors */
    class Source {
    public:
        std::string filename;
        int line;
        int character;

        Source() : line(0), character(0) {}

        void set(__const__ TextInput& ti, __const__ Token& t) {
            filename = ti.filename();
            line = t.line();
            character = t.character();
        }
    };

    typedef Array<Any> AnyArray;
    typedef Table<std::string, Any> AnyTable;

private:

    /** Called from deserialize() */
    static void deserializeComment(TextInput& ti, Token& token, std::string& comment);

    /** NONE, BOOLEAN, and NUMBER are stored directly in the Any */
    union SimpleValue {
        bool b;
        double n;

        __inline__ SimpleValue() : n(0.0) {}
        __inline__ SimpleValue(bool x) : b(x) {}
        __inline__ SimpleValue(double x) : n(x) {}
    };

    class Data {
    public:
        /** ARRAY, TABLE, or STRING value only.  NULL otherwise. */
        union Value {
            std::string* s;
            Array<Any>* a;
            AnyTable* t;
            __inline__ Value() : s(__null) {}
        };

        // Needed so that the destructor knows what is in Value
        // and can call its destructor. 
        Type type;

        /** Always points to memory that is allocated with the Data, so
           the destructor does not delete this. */
        Value value;

        std::string comment;

        std::string name;

        /** For STRING, ARRAY and TABLE types, m_value is shared between
            multiple instances.  Mutation is allowed only if the reference
            count is exactly 1, otherwise the mutating instance must copy
            the value.  This is not used for other types.
        */
        AtomicInt32 referenceCount;

        Source source;

    private:

        /** Called by create() */
        __inline__ Data(Type t) : type(t), referenceCount(1) {}

        /** Called by destroy */
        ~Data();

    public:

        /** Clones the argument */
        static Data* create(__const__ Data* d);
        static Data* create(Type t);

        /** Free d, invoking its destructor and freeing the memory for
            the value. */
        static void destroy(Data* d);

    };

    /** If not empty, this Any was created from operator[] on a table
        and perhaps was not intended to exist.  The name is needed to
        format the error message if it is read from before it is
        written to.

        The source of a placeholder object is that of the parent
        object until it is written.
    */
    std::string m_placeholderName;

    Type m_type;
    SimpleValue m_simpleValue;
    mutable Data* m_data;

    /** Called before every read operation to ensure that this object
        is not a placeholder.  */
    void beforeRead() __const__;

    /** Called before every write operation to wipe the placeholder
        status. */
    void beforeWrite();

    /** Decrements the reference count (if there is one).  If the
    reference count is zero after decrement, calls delete on @a m_data
    and sets it to NULL.
    */
    void dropReference();

    /** Allocate the Data object if it does not exist */
    void ensureData();

    /** If m_data is not NULL, ensure that it has a unique reference
        and contains a valid m_data.  This has a race condition if two
        threads are both trying to modify the same Any
        simultaneously.*/
    void ensureMutable();

    /** Read an unnamed a TABLE or ARRAY.  Token should be the open
        paren token; it is the next token after the close on
        return. Called from deserialize().*/
    void deserializeBody(TextInput& ti, Token& token);

    void deserialize(TextInput& ti, Token& token);

    /** Read the name of a named Array or Table. */
    static void deserializeName(TextInput& ti, Token& token, std::string& name);

    /** Read until a comma is consumed or a close paren is hit, and
     return that token.  Considers the passed in token to be the first
     value read. */
    static void readUntilCommaOrClose(TextInput& ti, Token& token);

    /** Construct an Any that is a proxy for a table fetch from \a data.
     This proxy can be copied exactly once on return from operator[].*/
    Any(__const__ std::string& key, Data* data);

    __inline__ bool isPlaceholder() __const__ {
        return ! m_placeholderName.empty();
    }

public:

    /** Base class for all Any exceptions.*/
    class Exception {
    public:
        virtual ~Exception() {}
    };

    /** Thrown by operator[] when a key is not present in a const table. */
    class KeyNotFound : public ParseError {
    public:
        std::string key;
    };

    /** Thrown by operator[] when an array index is not present. */
    class IndexOutOfBounds : public Exception {
    public:
        int index;
        int size;
        __inline__ IndexOutOfBounds() : index(0), size(0) {}
        __inline__ IndexOutOfBounds(int i, int s) : index(i), size(s) {}
    };

    /** NONE constructor */
    Any();

    /** Deserialize */
    explicit Any(TextInput& t);

    Any(__const__ Any& x);

    /** NUMBER constructor */
    Any(double x);


    /** NUMBER constructor */
    Any(int64 x);







    /** NUMBER constructor */
    Any(long x);

    /** NUMBER constructor */
    Any(int x);

    /** NUMBER constructor */
    Any(short x);

    /** BOOLEAN constructor */
    Any(bool x);

    /** STRING constructor */
    Any(__const__ std::string& x);

    /** STRING constructor */
    Any(__const__ char* x);

    /** \a t must be ARRAY or TABLE */
    Any(Type t, __const__ std::string& name = "");

    ~Any();

    /** Removes the comment and name */
    Any& operator=(__const__ Any& x);

    /** Removes the comment and name */
    Any& operator=(double x);

    /** Removes the comment and name */
    Any& operator=(int x);

    /** Removes the comment and name */
    Any& operator=(bool x);

    /** Removes the comment and name */
    Any& operator=(__const__ std::string& x);

    /** Removes the comment and name */
    Any& operator=(__const__ char* x);

    /** \a t must be ARRAY, TABLE, or NONE. Removes the comment and name */
    Any& operator=(Type t);

    Type type() __const__;

    /** Same as deserialize or load, but operates on a string instead
        of a stream or file.

      \sa deserialize, load
      */
    void parse(__const__ std::string& src);

    std::string unparse() __const__;

    /** Comments appear before values when they are in serialized form.*/
    __const__ std::string& comment() __const__;
    void setComment(__const__ std::string& c);

    /** True if this is the NONE value */
    bool isNone() __const__;

    /** Throws a ParseError exception if this is not a number */
    double number() __const__;
    __const__ std::string& string() __const__;
    bool boolean() __const__;

    /** If a valid string, takes the string value and creates a fully qualified filename.
        If not found, the returned string is empty.

        The file is searched for the following ways:
        
        - In the directory from which the Any was loaded.
        - By calling System::findDataFile as you would with other data files.
     */
    std::string resolveStringAsFilename() __const__;

    /** If this is named ARRAY or TABLE, returns the name. */
    __const__ std::string& name() __const__;

    /** If this is named ARRAY or TABLE, returns true if the name begins with \a s.  The comparision is case insensitive. */
    bool nameBeginsWith(__const__ std::string& s) __const__;

    /** If this is named ARRAY or TABLE, returns true if the name begins with \a s.  The comparision is case insensitive. */
    bool nameBeginsWith(__const__ char* s) __const__;

    /** If this is named ARRAY or TABLE, returns true if the name is \a s.  The comparision is case insensitive. */
    bool nameEquals(__const__ std::string& s) __const__;

    /** If this is named ARRAY or TABLE, returns true if the name is\a s.  The comparision is case insensitive. */
    bool nameEquals(__const__ char* s) __const__;

    /** \brief Set the name used when serializing an ARRAY or TABLE.
    
       Only legal for ARRAY or TABLE.  The \a name must begin with a letter
       and contain only letters, numbers, underscores and scope operators.

        <pre>
        a2z
        hello
        Foo::bar
        color.red
        this->that
        __x
        </pre>
        
                
        The scope operators "::", "->", and
        ".", may have spaces around them.  The name may not
        contain parentheses.
    */
    void setName(__const__ std::string& name);

    /** Number of elements if this is an ARRAY or TABLE */
    int size() __const__;
    int length() __const__;

    /** For an array, returns the ith element */
    __const__ Any& operator[](int i) __const__;
    Any& operator[](int i);

    __const__ Any& last() __const__ {
        return (*this)[size() - 1];
    }

    Any& last() {
        return (*this)[size() - 1];
    }

    /** Directly exposes the underlying data structure for an ARRAY. */
    __const__ Array<Any>& array() __const__;
    void append(__const__ Any& v0);
    void append(__const__ Any& v0, __const__ Any& v1);
    void append(__const__ Any& v0, __const__ Any& v1, __const__ Any& v2);
    void append(__const__ Any& v0, __const__ Any& v1, __const__ Any& v2, __const__ Any& v3);

    /** Directly exposes the underlying data structure for table.*/
    __const__ Table<std::string, Any>& table() __const__;

    /** For a table, returns the element for \a key. Throws KeyNotFound
        exception if the element does not exist.
       */
    __const__ Any& operator[](__const__ std::string& key) __const__;

    // Needed to prevent the operator[](int) overload from catching
    // string literals
    __const__ Any& operator[](__const__ char* key) __const__ {
        return operator[](std::string(key));
    }

    /** 
        Fetch an element from a table.  This can be used as:

        <pre>
        a["key"] = value;  (create the key if it did not exist)
        </pre>
        
        or

        <pre>
        value = a["key"];  (throw an error if the key did not exist)
        </pre>

        <b>Note:</b>
        In order to cause elements to be correctly created in the
        first case while still providing "key not found" errors in the
        second case, the Any returned is a special object that delays
        the actual fetch until the following assignment or method
        call.  This means that in the event of an error, the exception
        may be thrown from a line other than the actual fetch.  Use
        the Any::get() or the const Any::operator[]() methods to avoid
        this behavior and ensure error-checking at fetch time.
     */
    Any& operator[](__const__ std::string& key);

    /** \copydoc Any::operator[](const std::string&) */
    Any& operator[](__const__ char* key) {
        return operator[](std::string(key));
    }

    /** For a table, returns the element for key \a x and \a
        defaultVal if it does not exist. */
    __const__ Any& get(__const__ std::string& key, __const__ Any& defaultVal) __const__;

    /** Returns true if this key is in the TABLE.  Illegal to call on an object that is not a TABLE. */
    bool containsKey(__const__ std::string& key) __const__;

    /** For a table, assigns the element for key k. */
    void set(__const__ std::string& key, __const__ Any& val);

    /** for an ARRAY, resizes and returns the last element */
    Any& next();

    /** The parent directory of the location from which this Any was loaded.  This is useful for 
       interpreting filenames relative to the Any's source location,
       which may not match the current directory if the Any was from an included file. */
    std::string sourceDirectory() __const__;

    /** True if the Anys are exactly equal, ignoring comments.  Applies deeply on arrays and tables. */
    bool operator==(__const__ Any& x) __const__;
    bool operator!=(__const__ Any& x) __const__;

    operator int() __const__;
    operator float() __const__;
    operator double() __const__;
    operator bool() __const__;
    operator std::string() __const__;

    /** Resize to \a n elements, where new elements are NIL 
       It is an error to call this method if this is not an Any::ARRAY */
    void resize(int n);

    /** 
       Clears all entries.
       This must be a TABLE or ARRAY */
    void clear();

    /** Parse from a file.
     \sa deserialize, parse */
    void load(__const__ std::string& filename);

    /** Uses the serialize method. */
    void save(__const__ std::string& filename) __const__;

    void serialize(TextOutput& to) __const__;
    /** Parse from a stream.
     \sa load, parse */
    void deserialize(TextInput& ti);

    __const__ Source& source() __const__;

    /** Throws a ParseError if \a value is false.  Useful for quickly
        creating parse rules in classes that deserialize from Any.
    */
    void verify(bool value, __const__ std::string& message = "") __const__;


    /** Verifies that the name <i>begins with</i> identifier \a n (case insensitive). 
        It may contain identifier operators after this */
    void verifyName(__const__ std::string& n) __const__;

    /** Verifies that the name <i>begins with</i> identifier \a n or \a m (case insensitive). 
        It may contain identifier operators after this */
    void verifyName(__const__ std::string& n, __const__ std::string& m) __const__;

    /** Verifies that the type is \a t. */
    void verifyType(Type t) __const__;

    /** Throws an exception if the type is not \a t0 or \a t1. */
    void verifyType(Type t0, Type t1) __const__;

    /** Verifies that the size is between \a low and \a high, inclusive */
    void verifySize(int low, int high) __const__;

    /** Verifies that the size is exactly \a s */
    void verifySize(int s) __const__;

private:

    void deserializeTable(TextInput& ti);
    void deserializeArray(TextInput& ti,__const__ std::string& term);

}; // class Any


/**
   Convenient iteration over the keys of a Any::TABLE, usually
   for implementing construction of an object from an Any.

   Getting an element using either iteration or explicit requests
   consumes that element from the iterator (but not from the Any!)
   It is an error to consume the same element more than once from
   the same iterator.

   <pre>
    AnyKeyIterator r(a);
    r.getIfPresent("enabled",            enabled);
    r.getIfPresent("showSamples",        showSamples);
    r.getIfPresent("showTiles",          showTiles);

    r.verifyDone();
    </pre>

    \beta
*/
class AnyTableReader {
private:
   Any m_any;
   Set<std::string> m_alreadyRead;
public:

    /** Verifies that \a is a TABLE with the given \a name. */
    AnyTableReader(__const__ std::string& name, __const__ Any& a) : m_any(a) {
        try {
            m_any.verifyType(Any::TABLE);
            m_any.verifyName(name);
        } catch (__const__ ParseError& e) {
            // If an exception is thrown, the destructors will not be 
            // invoked automatically.
            m_any.~Any();
            m_alreadyRead.~Set();
            throw e;
        }
    }

    /** Verifies that \a is a TABLE. */
    AnyTableReader(__const__ Any& a) : m_any(a) {
        try {
            m_any.verifyType(Any::TABLE);
        } catch (__const__ ParseError& e) {
            // If an exception is thrown, the destructors will not be 
            // invoked automatically.
            m_any.~Any();
            m_alreadyRead.~Set();
            throw e;
        }
    }

    bool hasMore() __const__ {
        return m_any.size() > m_alreadyRead.size();
    }

    /** Verifies that all keys have been read. */
    void verifyDone() __const__ {
        if (hasMore()) {
            // Generate all keys
            // Remove the ones we've read
            // Assert the rest
          //  any.verify("");
        }
    }
# 718 "/usr/local/include/G3D/Any.h" 3
    /** If key \s appears in the any, reads its value into \a v and 
        removes that key from the ones available to iterate over.

        If key \s does not appear in the any, throws a G3D::ParseError.

        Assumes that if key \s appears in the any it has not already been extracted
        by this iterator.  If it has been read before, an assertion will fail in debug mode.

      */
    template<class ValueType>
    void get(__const__ std::string& s, ValueType& v) {
        v = m_any[s];
        m_alreadyRead.insert(toLower(s));
    }

    /** Get the value associated with a key only if the key is actually present.
    
        If key \s appears in the any, reads its value into \a v and 
        removes that key from the ones available to iterate over.

        If key \s does not appear in the any, does nothing.

        Assumes that if key \s appears in the any it has not already been extracted
        by this iterator.  If it has been read before, an assertion will fail in debug mode.

        \return True if the value was read.
      */
    template<class ValueType>
    bool getIfPresent(__const__ std::string& s, ValueType& v) {
        if (m_any.containsKey(s)) {
            do {} while (0);

            get(s, v);
            return true;
        } else {
            return false;
        }
    }
};

} // namespace G3D
