# 45 "/usr/include/c++/4.5/streambuf" 2 3

namespace std __attribute__ ((__visibility__ ("default"))) {

  template<typename _CharT, typename _Traits>
    streamsize
    __copy_streambufs_eof(basic_streambuf<_CharT, _Traits>*,
     basic_streambuf<_CharT, _Traits>*, bool&);

  /**
   *  @brief  The actual work of input and output (interface).
   *  @ingroup io
   *
   *  This is a base class.  Derived stream buffers each control a
   *  pair of character sequences:  one for input, and one for output.
   *
   *  Section [27.5.1] of the standard describes the requirements and
   *  behavior of stream buffer classes.  That section (three paragraphs)
   *  is reproduced here, for simplicity and accuracy.
   *
   *  -# Stream buffers can impose various constraints on the sequences
   *     they control.  Some constraints are:
   *     - The controlled input sequence can be not readable.
   *     - The controlled output sequence can be not writable.
   *     - The controlled sequences can be associated with the contents of
   *       other representations for character sequences, such as external
   *       files.
   *     - The controlled sequences can support operations @e directly to or
   *       from associated sequences.
   *     - The controlled sequences can impose limitations on how the
   *       program can read characters from a sequence, write characters to
   *       a sequence, put characters back into an input sequence, or alter
   *       the stream position.
   *     .
   *  -# Each sequence is characterized by three pointers which, if non-null,
   *     all point into the same @c charT array object.  The array object
   *     represents, at any moment, a (sub)sequence of characters from the
   *     sequence.  Operations performed on a sequence alter the values
   *     stored in these pointers, perform reads and writes directly to or
   *     from associated sequences, and alter <em>the stream position</em> and
   *     conversion state as needed to maintain this subsequence relationship.
   *     The three pointers are:
   *     - the <em>beginning pointer</em>, or lowest element address in the
   *       array (called @e xbeg here);
   *     - the <em>next pointer</em>, or next element address that is a
   *       current candidate for reading or writing (called @e xnext here);
   *     - the <em>end pointer</em>, or first element address beyond the
   *       end of the array (called @e xend here).
   *     .
   *  -# The following semantic constraints shall always apply for any set
   *     of three pointers for a sequence, using the pointer names given
   *     immediately above:
   *     - If @e xnext is not a null pointer, then @e xbeg and @e xend shall
   *       also be non-null pointers into the same @c charT array, as
   *       described above; otherwise, @e xbeg and @e xend shall also be null.
   *     - If @e xnext is not a null pointer and @e xnext < @e xend for an
   *       output sequence, then a <em>write position</em> is available.
   *       In this case, @e *xnext shall be assignable as the next element
   *       to write (to put, or to store a character value, into the sequence).
   *     - If @e xnext is not a null pointer and @e xbeg < @e xnext for an
   *       input sequence, then a <em>putback position</em> is available.
   *       In this case, @e xnext[-1] shall have a defined value and is the
   *       next (preceding) element to store a character that is put back
   *       into the input sequence.
   *     - If @e xnext is not a null pointer and @e xnext< @e xend for an
   *       input sequence, then a <em>read position</em> is available.
   *       In this case, @e *xnext shall have a defined value and is the
   *       next element to read (to get, or to obtain a character value,
   *       from the sequence).
  */
  template<typename _CharT, typename _Traits>
    class basic_streambuf
    {
    public:
      //@{
      /**
       *  These are standard types.  They permit a standardized way of
       *  referring to names of (or names dependant on) the template
       *  parameters, which are specific to the implementation.
      */
      typedef _CharT char_type;
      typedef _Traits traits_type;
      typedef typename traits_type::int_type int_type;
      typedef typename traits_type::pos_type pos_type;
      typedef typename traits_type::off_type off_type;
      //@}

      //@{
      /// This is a non-standard type.
      typedef basic_streambuf<char_type, traits_type> __streambuf_type;
      //@}

      friend class basic_ios<char_type, traits_type>;
      friend class basic_istream<char_type, traits_type>;
      friend class basic_ostream<char_type, traits_type>;
      friend class istreambuf_iterator<char_type, traits_type>;
      friend class ostreambuf_iterator<char_type, traits_type>;

      friend streamsize
      __copy_streambufs_eof<>(__streambuf_type*, __streambuf_type*, bool&);

      template<bool _IsMove, typename _CharT2>
        friend typename __gnu_cxx::__enable_if<__is_char<_CharT2>::__value,
            _CharT2*>::__type
        __copy_move_a2(istreambuf_iterator<_CharT2>,
         istreambuf_iterator<_CharT2>, _CharT2*);

      template<typename _CharT2>
        friend typename __gnu_cxx::__enable_if<__is_char<_CharT2>::__value,
      istreambuf_iterator<_CharT2> >::__type
        find(istreambuf_iterator<_CharT2>, istreambuf_iterator<_CharT2>,
      __const__ _CharT2&);

      template<typename _CharT2, typename _Traits2>
        friend basic_istream<_CharT2, _Traits2>&
        operator>>(basic_istream<_CharT2, _Traits2>&, _CharT2*);

      template<typename _CharT2, typename _Traits2, typename _Alloc>
        friend basic_istream<_CharT2, _Traits2>&
        operator>>(basic_istream<_CharT2, _Traits2>&,
     basic_string<_CharT2, _Traits2, _Alloc>&);

      template<typename _CharT2, typename _Traits2, typename _Alloc>
        friend basic_istream<_CharT2, _Traits2>&
        getline(basic_istream<_CharT2, _Traits2>&,
  basic_string<_CharT2, _Traits2, _Alloc>&, _CharT2);

    protected:
      //@{
      /**
       *  This is based on _IO_FILE, just reordered to be more consistent,
       *  and is intended to be the most minimal abstraction for an
       *  internal buffer.
       *  -  get == input == read
       *  -  put == output == write
      */
      char_type* _M_in_beg; // Start of get area. 
      char_type* _M_in_cur; // Current read area. 
      char_type* _M_in_end; // End of get area. 
      char_type* _M_out_beg; // Start of put area. 
      char_type* _M_out_cur; // Current put area. 
      char_type* _M_out_end; // End of put area.

      /// Current locale setting.
      locale _M_buf_locale;

  public:
      /// Destructor deallocates no buffer space.
      virtual
      ~basic_streambuf()
      { }

      // [27.5.2.2.1] locales
      /**
       *  @brief  Entry point for imbue().
       *  @param  loc  The new locale.
       *  @return  The previous locale.
       *
       *  Calls the derived imbue(loc).
      */
      locale
      pubimbue(__const__ locale &__loc)
      {
 locale __tmp(this->getloc());
 this->imbue(__loc);
 _M_buf_locale = __loc;
 return __tmp;
      }

      /**
       *  @brief  Locale access.
       *  @return  The current locale in effect.
       *
       *  If pubimbue(loc) has been called, then the most recent @c loc
       *  is returned.  Otherwise the global locale in effect at the time
       *  of construction is returned.
      */
      locale
      getloc() __const__
      { return _M_buf_locale; }

      // [27.5.2.2.2] buffer management and positioning
      //@{
      /**
       *  @brief  Entry points for derived buffer functions.
       *
       *  The public versions of @c pubfoo dispatch to the protected
       *  derived @c foo member functions, passing the arguments (if any)
       *  and returning the result unchanged.
      */
      __streambuf_type*
      pubsetbuf(char_type* __s, streamsize __n)
      { return this->setbuf(__s, __n); }

      pos_type
      pubseekoff(off_type __off, ios_base::seekdir __way,
   ios_base::openmode __mode = ios_base::in | ios_base::out)
      { return this->seekoff(__off, __way, __mode); }

      pos_type
      pubseekpos(pos_type __sp,
   ios_base::openmode __mode = ios_base::in | ios_base::out)
      { return this->seekpos(__sp, __mode); }

      int
      pubsync() { return this->sync(); }
      //@}

      // [27.5.2.2.3] get area
      /**
       *  @brief  Looking ahead into the stream.
       *  @return  The number of characters available.
       *
       *  If a read position is available, returns the number of characters
       *  available for reading before the buffer must be refilled.
       *  Otherwise returns the derived @c showmanyc().
      */
      streamsize
      in_avail()
      {
 __const__ streamsize __ret = this->egptr() - this->gptr();
 return __ret ? __ret : this->showmanyc();
      }

      /**
       *  @brief  Getting the next character.
       *  @return  The next character, or eof.
       *
       *  Calls @c sbumpc(), and if that function returns
       *  @c traits::eof(), so does this function.  Otherwise, @c sgetc().
      */
      int_type
      snextc()
      {
 int_type __ret = traits_type::eof();
 if (__builtin_expect(!traits_type::eq_int_type(this->sbumpc(),
             __ret), true))
   __ret = this->sgetc();
 return __ret;
      }

      /**
       *  @brief  Getting the next character.
       *  @return  The next character, or eof.
       *
       *  If the input read position is available, returns that character
       *  and increments the read pointer, otherwise calls and returns
       *  @c uflow().
      */
      int_type
      sbumpc()
      {
 int_type __ret;
 if (__builtin_expect(this->gptr() < this->egptr(), true))
   {
     __ret = traits_type::to_int_type(*this->gptr());
     this->gbump(1);
   }
 else
   __ret = this->uflow();
 return __ret;
      }

      /**
       *  @brief  Getting the next character.
       *  @return  The next character, or eof.
       *
       *  If the input read position is available, returns that character,
       *  otherwise calls and returns @c underflow().  Does not move the 
       *  read position after fetching the character.
      */
      int_type
      sgetc()
      {
 int_type __ret;
 if (__builtin_expect(this->gptr() < this->egptr(), true))
   __ret = traits_type::to_int_type(*this->gptr());
 else
   __ret = this->underflow();
 return __ret;
      }

      /**
       *  @brief  Entry point for xsgetn.
       *  @param  s  A buffer area.
       *  @param  n  A count.
       *
       *  Returns xsgetn(s,n).  The effect is to fill @a s[0] through
       *  @a s[n-1] with characters from the input sequence, if possible.
      */
      streamsize
      sgetn(char_type* __s, streamsize __n)
      { return this->xsgetn(__s, __n); }

      // [27.5.2.2.4] putback
      /**
       *  @brief  Pushing characters back into the input stream.
       *  @param  c  The character to push back.
       *  @return  The previous character, if possible.
       *
       *  Similar to sungetc(), but @a c is pushed onto the stream
       *  instead of <em>the previous character.</em> If successful,
       *  the next character fetched from the input stream will be @a
       *  c.
      */
      int_type
      sputbackc(char_type __c)
      {
 int_type __ret;
 __const__ bool __testpos = this->eback() < this->gptr();
 if (__builtin_expect(!__testpos ||
        !traits_type::eq(__c, this->gptr()[-1]), false))
   __ret = this->pbackfail(traits_type::to_int_type(__c));
 else
   {
     this->gbump(-1);
     __ret = traits_type::to_int_type(*this->gptr());
   }
 return __ret;
      }

      /**
       *  @brief  Moving backwards in the input stream.
       *  @return  The previous character, if possible.
       *
       *  If a putback position is available, this function decrements
       *  the input pointer and returns that character.  Otherwise,
       *  calls and returns pbackfail().  The effect is to @a unget
       *  the last character @a gotten.
      */
      int_type
      sungetc()
      {
 int_type __ret;
 if (__builtin_expect(this->eback() < this->gptr(), true))
   {
     this->gbump(-1);
     __ret = traits_type::to_int_type(*this->gptr());
   }
 else
   __ret = this->pbackfail();
 return __ret;
      }

      // [27.5.2.2.5] put area
      /**
       *  @brief  Entry point for all single-character output functions.
       *  @param  c  A character to output.
       *  @return  @a c, if possible.
       *
       *  One of two public output functions.
       *
       *  If a write position is available for the output sequence (i.e.,
       *  the buffer is not full), stores @a c in that position, increments
       *  the position, and returns @c traits::to_int_type(c).  If a write
       *  position is not available, returns @c overflow(c).
      */
      int_type
      sputc(char_type __c)
      {
 int_type __ret;
 if (__builtin_expect(this->pptr() < this->epptr(), true))
   {
     *this->pptr() = __c;
     this->pbump(1);
     __ret = traits_type::to_int_type(__c);
   }
 else
   __ret = this->overflow(traits_type::to_int_type(__c));
 return __ret;
      }

      /**
       *  @brief  Entry point for all single-character output functions.
       *  @param  s  A buffer read area.
       *  @param  n  A count.
       *
       *  One of two public output functions.
       *
       *
       *  Returns xsputn(s,n).  The effect is to write @a s[0] through
       *  @a s[n-1] to the output sequence, if possible.
      */
      streamsize
      sputn(__const__ char_type* __s, streamsize __n)
      { return this->xsputn(__s, __n); }

    protected:
      /**
       *  @brief  Base constructor.
       *
       *  Only called from derived constructors, and sets up all the
       *  buffer data to zero, including the pointers described in the
       *  basic_streambuf class description.  Note that, as a result,
       *  - the class starts with no read nor write positions available,
       *  - this is not an error
      */
      basic_streambuf()
      : _M_in_beg(0), _M_in_cur(0), _M_in_end(0),
      _M_out_beg(0), _M_out_cur(0), _M_out_end(0),
      _M_buf_locale(locale())
      { }

      // [27.5.2.3.1] get area access
      //@{
      /**
       *  @brief  Access to the get area.
       *
       *  These functions are only available to other protected functions,
       *  including derived classes.
       *
       *  - eback() returns the beginning pointer for the input sequence
       *  - gptr() returns the next pointer for the input sequence
       *  - egptr() returns the end pointer for the input sequence
      */
      char_type*
      eback() __const__ { return _M_in_beg; }

      char_type*
      gptr() __const__ { return _M_in_cur; }

      char_type*
      egptr() __const__ { return _M_in_end; }
      //@}

      /**
       *  @brief  Moving the read position.
       *  @param  n  The delta by which to move.
       *
       *  This just advances the read position without returning any data.
      */
      void
      gbump(int __n) { _M_in_cur += __n; }

      /**
       *  @brief  Setting the three read area pointers.
       *  @param  gbeg  A pointer.
       *  @param  gnext  A pointer.
       *  @param  gend  A pointer.
       *  @post  @a gbeg == @c eback(), @a gnext == @c gptr(), and
       *         @a gend == @c egptr()
      */
      void
      setg(char_type* __gbeg, char_type* __gnext, char_type* __gend)
      {
 _M_in_beg = __gbeg;
 _M_in_cur = __gnext;
 _M_in_end = __gend;
      }

      // [27.5.2.3.2] put area access
      //@{
      /**
       *  @brief  Access to the put area.
       *
       *  These functions are only available to other protected functions,
       *  including derived classes.
       *
       *  - pbase() returns the beginning pointer for the output sequence
       *  - pptr() returns the next pointer for the output sequence
       *  - epptr() returns the end pointer for the output sequence
      */
      char_type*
      pbase() __const__ { return _M_out_beg; }

      char_type*
      pptr() __const__ { return _M_out_cur; }

      char_type*
      epptr() __const__ { return _M_out_end; }
      //@}

      /**
       *  @brief  Moving the write position.
       *  @param  n  The delta by which to move.
       *
       *  This just advances the write position without returning any data.
      */
      void
      pbump(int __n) { _M_out_cur += __n; }

      /**
       *  @brief  Setting the three write area pointers.
       *  @param  pbeg  A pointer.
       *  @param  pend  A pointer.
       *  @post  @a pbeg == @c pbase(), @a pbeg == @c pptr(), and
       *         @a pend == @c epptr()
      */
      void
      setp(char_type* __pbeg, char_type* __pend)
      {
 _M_out_beg = _M_out_cur = __pbeg;
 _M_out_end = __pend;
      }

      // [27.5.2.4] virtual functions
      // [27.5.2.4.1] locales
      /**
       *  @brief  Changes translations.
       *  @param  loc  A new locale.
       *
       *  Translations done during I/O which depend on the current
       *  locale are changed by this call.  The standard adds,
       *  <em>Between invocations of this function a class derived
       *  from streambuf can safely cache results of calls to locale
       *  functions and to members of facets so obtained.</em>
       *
       *  @note  Base class version does nothing.
      */
      virtual void
      imbue(__const__ locale&)
      { }

      // [27.5.2.4.2] buffer management and positioning
      /**
       *  @brief  Manipulates the buffer.
       *
       *  Each derived class provides its own appropriate behavior.  See
       *  the next-to-last paragraph of 
       *  http://gcc.gnu.org/onlinedocs/libstdc++/manual/bk01pt11ch25s02.html
       *  for more on this function.
       *
       *  @note  Base class version does nothing, returns @c this.
      */
      virtual basic_streambuf<char_type,_Traits>*
      setbuf(char_type*, streamsize)
      { return this; }

      /**
       *  @brief  Alters the stream positions.
       *
       *  Each derived class provides its own appropriate behavior.
       *  @note  Base class version does nothing, returns a @c pos_type
       *         that represents an invalid stream position.
      */
      virtual pos_type
      seekoff(off_type, ios_base::seekdir,
       ios_base::openmode /*__mode*/ = ios_base::in | ios_base::out)
      { return pos_type(off_type(-1)); }

      /**
       *  @brief  Alters the stream positions.
       *
       *  Each derived class provides its own appropriate behavior.
       *  @note  Base class version does nothing, returns a @c pos_type
       *         that represents an invalid stream position.
      */
      virtual pos_type
      seekpos(pos_type,
       ios_base::openmode /*__mode*/ = ios_base::in | ios_base::out)
      { return pos_type(off_type(-1)); }

      /**
       *  @brief  Synchronizes the buffer arrays with the controlled sequences.
       *  @return  -1 on failure.
       *
       *  Each derived class provides its own appropriate behavior,
       *  including the definition of @a failure.
       *  @note  Base class version does nothing, returns zero.
      */
      virtual int
      sync() { return 0; }

      // [27.5.2.4.3] get area
      /**
       *  @brief  Investigating the data available.
       *  @return  An estimate of the number of characters available in the
       *           input sequence, or -1.
       *
       *  <em>If it returns a positive value, then successive calls to
       *  @c underflow() will not return @c traits::eof() until at
       *  least that number of characters have been supplied.  If @c
       *  showmanyc() returns -1, then calls to @c underflow() or @c
       *  uflow() will fail.</em> [27.5.2.4.3]/1
       *
       *  @note  Base class version does nothing, returns zero.
       *  @note  The standard adds that <em>the intention is not only that the
       *         calls [to underflow or uflow] will not return @c eof() but
       *         that they will return immediately.</em>
       *  @note  The standard adds that <em>the morphemes of @c showmanyc are
       *         @b es-how-many-see, not @b show-manic.</em>
      */
      virtual streamsize
      showmanyc() { return 0; }

      /**
       *  @brief  Multiple character extraction.
       *  @param  s  A buffer area.
       *  @param  n  Maximum number of characters to assign.
       *  @return  The number of characters assigned.
       *
       *  Fills @a s[0] through @a s[n-1] with characters from the input
       *  sequence, as if by @c sbumpc().  Stops when either @a n characters
       *  have been copied, or when @c traits::eof() would be copied.
       *
       *  It is expected that derived classes provide a more efficient
       *  implementation by overriding this definition.
      */
      virtual streamsize
      xsgetn(char_type* __s, streamsize __n);

      /**
       *  @brief  Fetches more data from the controlled sequence.
       *  @return  The first character from the <em>pending sequence</em>.
       *
       *  Informally, this function is called when the input buffer is
       *  exhausted (or does not exist, as buffering need not actually be
       *  done).  If a buffer exists, it is @a refilled.  In either case, the
       *  next available character is returned, or @c traits::eof() to
       *  indicate a null pending sequence.
       *
       *  For a formal definition of the pending sequence, see a good text
       *  such as Langer & Kreft, or [27.5.2.4.3]/7-14.
       *
       *  A functioning input streambuf can be created by overriding only
       *  this function (no buffer area will be used).  For an example, see
       *  http://gcc.gnu.org/onlinedocs/libstdc++/manual/bk01pt11ch25.html
       *
       *  @note  Base class version does nothing, returns eof().
      */
      virtual int_type
      underflow()
      { return traits_type::eof(); }

      /**
       *  @brief  Fetches more data from the controlled sequence.
       *  @return  The first character from the <em>pending sequence</em>.
       *
       *  Informally, this function does the same thing as @c underflow(),
       *  and in fact is required to call that function.  It also returns
       *  the new character, like @c underflow() does.  However, this
       *  function also moves the read position forward by one.
      */
      virtual int_type
      uflow()
      {
 int_type __ret = traits_type::eof();
 __const__ bool __testeof = traits_type::eq_int_type(this->underflow(),
       __ret);
 if (!__testeof)
   {
     __ret = traits_type::to_int_type(*this->gptr());
     this->gbump(1);
   }
 return __ret;
      }

      // [27.5.2.4.4] putback
      /**
       *  @brief  Tries to back up the input sequence.
       *  @param  c  The character to be inserted back into the sequence.
       *  @return  eof() on failure, <em>some other value</em> on success
       *  @post  The constraints of @c gptr(), @c eback(), and @c pptr()
       *         are the same as for @c underflow().
       *
       *  @note  Base class version does nothing, returns eof().
      */
      virtual int_type
      pbackfail(int_type /* __c */ = traits_type::eof())
      { return traits_type::eof(); }

      // Put area:
      /**
       *  @brief  Multiple character insertion.
       *  @param  s  A buffer area.
       *  @param  n  Maximum number of characters to write.
       *  @return  The number of characters written.
       *
       *  Writes @a s[0] through @a s[n-1] to the output sequence, as if
       *  by @c sputc().  Stops when either @a n characters have been
       *  copied, or when @c sputc() would return @c traits::eof().
       *
       *  It is expected that derived classes provide a more efficient
       *  implementation by overriding this definition.
      */
      virtual streamsize
      xsputn(__const__ char_type* __s, streamsize __n);

      /**
       *  @brief  Consumes data from the buffer; writes to the
       *          controlled sequence.
       *  @param  c  An additional character to consume.
       *  @return  eof() to indicate failure, something else (usually
       *           @a c, or not_eof())
       *
       *  Informally, this function is called when the output buffer
       *  is full (or does not exist, as buffering need not actually
       *  be done).  If a buffer exists, it is @a consumed, with
       *  <em>some effect</em> on the controlled sequence.
       *  (Typically, the buffer is written out to the sequence
       *  verbatim.)  In either case, the character @a c is also
       *  written out, if @a c is not @c eof().
       *
       *  For a formal definition of this function, see a good text
       *  such as Langer & Kreft, or [27.5.2.4.5]/3-7.
       *
       *  A functioning output streambuf can be created by overriding only
       *  this function (no buffer area will be used).
       *
       *  @note  Base class version does nothing, returns eof().
      */
      virtual int_type
      overflow(int_type /* __c */ = traits_type::eof())
      { return traits_type::eof(); }


    // Annex D.6
    public:
      /**
       *  @brief  Tosses a character.
       *
       *  Advances the read pointer, ignoring the character that would have
       *  been read.
       *
       *  See http://gcc.gnu.org/ml/libstdc++/2002-05/msg00168.html
       */
      void
      stossc()
      {
 if (this->gptr() < this->egptr())
   this->gbump(1);
 else
   this->uflow();
      }


    private:
      // _GLIBCXX_RESOLVE_LIB_DEFECTS
      // Side effect of DR 50. 
      basic_streambuf(__const__ __streambuf_type& __sb)
      : _M_in_beg(__sb._M_in_beg), _M_in_cur(__sb._M_in_cur),
      _M_in_end(__sb._M_in_end), _M_out_beg(__sb._M_out_beg),
      _M_out_cur(__sb._M_out_cur), _M_out_end(__sb._M_out_cur),
      _M_buf_locale(__sb._M_buf_locale)
      { }

      __streambuf_type&
      operator=(__const__ __streambuf_type&) { return *this; };
    };

  // Explicit specialization declarations, defined in src/streambuf.cc.
  template<>
    streamsize
    __copy_streambufs_eof(basic_streambuf<char>* __sbin,
     basic_streambuf<char>* __sbout, bool& __ineof);

  template<>
    streamsize
    __copy_streambufs_eof(basic_streambuf<wchar_t>* __sbin,
     basic_streambuf<wchar_t>* __sbout, bool& __ineof);


}





# 39 "/usr/include/c++/4.5/bits/streambuf.tcc" 3

namespace std __attribute__ ((__visibility__ ("default"))) {

  template<typename _CharT, typename _Traits>
    streamsize
    basic_streambuf<_CharT, _Traits>::
    xsgetn(char_type* __s, streamsize __n)
    {
      streamsize __ret = 0;
      while (__ret < __n)
 {
   __const__ streamsize __buf_len = this->egptr() - this->gptr();
   if (__buf_len)
     {
       __const__ streamsize __remaining = __n - __ret;
       __const__ streamsize __len = std::min(__buf_len, __remaining);
       traits_type::copy(__s, this->gptr(), __len);
       __ret += __len;
       __s += __len;
       this->gbump(__len);
     }

   if (__ret < __n)
     {
       __const__ int_type __c = this->uflow();
       if (!traits_type::eq_int_type(__c, traits_type::eof()))
  {
    traits_type::assign(*__s++, traits_type::to_char_type(__c));
    ++__ret;
  }
       else
  break;
     }
 }
      return __ret;
    }

  template<typename _CharT, typename _Traits>
    streamsize
    basic_streambuf<_CharT, _Traits>::
    xsputn(__const__ char_type* __s, streamsize __n)
    {
      streamsize __ret = 0;
      while (__ret < __n)
 {
   __const__ streamsize __buf_len = this->epptr() - this->pptr();
   if (__buf_len)
     {
       __const__ streamsize __remaining = __n - __ret;
       __const__ streamsize __len = std::min(__buf_len, __remaining);
       traits_type::copy(this->pptr(), __s, __len);
       __ret += __len;
       __s += __len;
       this->pbump(__len);
     }

   if (__ret < __n)
     {
       int_type __c = this->overflow(traits_type::to_int_type(*__s));
       if (!traits_type::eq_int_type(__c, traits_type::eof()))
  {
    ++__ret;
    ++__s;
  }
       else
  break;
     }
 }
      return __ret;
    }

  // Conceivably, this could be used to implement buffer-to-buffer
  // copies, if this was ever desired in an un-ambiguous way by the
  // standard.
  template<typename _CharT, typename _Traits>
    streamsize
    __copy_streambufs_eof(basic_streambuf<_CharT, _Traits>* __sbin,
     basic_streambuf<_CharT, _Traits>* __sbout,
     bool& __ineof)
    {
      streamsize __ret = 0;
      __ineof = true;
      typename _Traits::int_type __c = __sbin->sgetc();
      while (!_Traits::eq_int_type(__c, _Traits::eof()))
 {
   __c = __sbout->sputc(_Traits::to_char_type(__c));
   if (_Traits::eq_int_type(__c, _Traits::eof()))
     {
       __ineof = false;
       break;
     }
   ++__ret;
   __c = __sbin->snextc();
 }
      return __ret;
    }

  template<typename _CharT, typename _Traits>
    __inline__ streamsize
    __copy_streambufs(basic_streambuf<_CharT, _Traits>* __sbin,
        basic_streambuf<_CharT, _Traits>* __sbout)
    {
      bool __ineof;
      return __copy_streambufs_eof(__sbin, __sbout, __ineof);
    }

  // Inhibit implicit instantiations for required instantiations,
  // which are defined via explicit instantiations elsewhere.
  // NB:  This syntax is a GNU extension.

  extern template class basic_streambuf<char>;
  extern template
    streamsize
    __copy_streambufs(basic_streambuf<char>*,
        basic_streambuf<char>*);
  extern template
    streamsize
    __copy_streambufs_eof(basic_streambuf<char>*,
     basic_streambuf<char>*, bool&);


  extern template class basic_streambuf<wchar_t>;
  extern template
    streamsize
    __copy_streambufs(basic_streambuf<wchar_t>*,
        basic_streambuf<wchar_t>*);
  extern template
    streamsize
    __copy_streambufs_eof(basic_streambuf<wchar_t>*,
     basic_streambuf<wchar_t>*, bool&);



}
