
namespace std __attribute__ ((__visibility__ ("default"))) {
# 62 "/usr/include/c++/4.5/bits/locale_classes.h" 3
  class locale
  {
  public:


    typedef int category;


    class facet;
    class id;
    class _Impl;

    friend class facet;
    friend class _Impl;

    template<typename _Facet>
      friend bool
      has_facet(__const__ locale&) throw();

    template<typename _Facet>
      friend __const__ _Facet&
      use_facet(__const__ locale&);

    template<typename _Cache>
      friend struct __use_cache;
# 98 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    static __const__ category none = 0;
    static __const__ category ctype = 1L << 0;
    static __const__ category numeric = 1L << 1;
    static __const__ category collate = 1L << 2;
    static __const__ category time = 1L << 3;
    static __const__ category monetary = 1L << 4;
    static __const__ category messages = 1L << 5;
    static __const__ category all = (ctype | numeric | collate |
        time | monetary | messages);
# 117 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    locale() throw();
# 126 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    locale(__const__ locale& __other) throw();
# 136 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    explicit
    locale(__const__ char* __s);
# 151 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    locale(__const__ locale& __base, __const__ char* __s, category __cat);
# 164 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    locale(__const__ locale& __base, __const__ locale& __add, category __cat);
# 176 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    template<typename _Facet>
      locale(__const__ locale& __other, _Facet* __f);


    ~locale() throw();
# 190 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    __const__ locale&
    operator=(__const__ locale& __other) throw();
# 205 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    template<typename _Facet>
      locale
      combine(__const__ locale& __other) __const__;






    string
    name() __const__;
# 224 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    bool
    operator==(__const__ locale& __other) __const__ throw();







    bool
    operator!=(__const__ locale& __other) __const__ throw()
    { return !(this->operator==(__other)); }
# 252 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    template<typename _Char, typename _Traits, typename _Alloc>
      bool
      operator()(__const__ basic_string<_Char, _Traits, _Alloc>& __s1,
   __const__ basic_string<_Char, _Traits, _Alloc>& __s2) __const__;
# 268 "/usr/include/c++/4.5/bits/locale_classes.h" 3
    static locale
    global(__const__ locale&);




    static __const__ locale&
    classic();


  };

