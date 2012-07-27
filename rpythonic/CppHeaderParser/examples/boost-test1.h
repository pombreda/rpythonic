namespace boost
{
    class thread_group:
        private noncopyable
    {
    public:
        ~thread_group()
        {
            for(std::list<thread*>::iterator it=threads.begin(),end=threads.end();
                it!=end;
                ++it)
            {
                delete *it;
            }
        }

        template<typename F>
        thread* create_thread(F threadfunc)
        {
            boost::lock_guard<shared_mutex> guard(m);
            std::auto_ptr<thread> new_thread(new thread(threadfunc));
            threads.push_back(new_thread.get());
            return new_thread.release();
        }

        void add_thread(thread* thrd)
        {
            if(thrd)
            {
                boost::lock_guard<shared_mutex> guard(m);
                threads.push_back(thrd);
            }
        }

        void remove_thread(thread* thrd)
        {
            boost::lock_guard<shared_mutex> guard(m);
            std::list<thread*>::iterator __const__ it=std::find(threads.begin(),threads.end(),thrd);
            if(it!=threads.end())
            {
                threads.erase(it);
            }
        }

        void join_all()
        {
            boost::shared_lock<shared_mutex> guard(m);

            for(std::list<thread*>::iterator it=threads.begin(),end=threads.end();
                it!=end;
                ++it)
            {
                (*it)->join();
            }
        }

        void interrupt_all()
        {
            boost::shared_lock<shared_mutex> guard(m);

            for(std::list<thread*>::iterator it=threads.begin(),end=threads.end();
                it!=end;
                ++it)
            {
                (*it)->interrupt();
            }
        }

        size_t size() __const__
        {
            boost::shared_lock<shared_mutex> guard(m);
            return threads.size();
        }

    private:
        std::list<thread*> threads;
        mutable shared_mutex m;
    };
}

