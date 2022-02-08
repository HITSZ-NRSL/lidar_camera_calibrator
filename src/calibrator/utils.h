/*
 * utils.h
 * Copyright (C) 2016 Nick.Liao <simplelife_nick@hotmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <cmath>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

//#define LQH_DEBUG


namespace lqh
{

const double MATH_PI = 3.1415926535897932384626433832;

namespace utils
{

/**
 * @brief
 * @note: used only in main thread
 */
class pm
{
private:
	boost::thread_group threadpool_;
	boost::asio::io_service service_;
	std::unique_ptr<boost::asio::io_service::work> work_;
public:
	// type
	class helper
	{
		friend class pm;
	private:
		std::mutex mtx_;
		std::condition_variable cv_;
		uint32_t cnt_;

		// we don't want them to be called outside
		void increase()
		{
			std::lock_guard<std::mutex> lg(mtx_);
			cnt_++;
		}

		void decrease()
		{
			std::unique_lock<std::mutex> lg(mtx_);
			cnt_--;

			lg.unlock();
			cv_.notify_one();
		}
		void wait()
		{
			std::unique_lock<std::mutex> ulk(mtx_);
			while(cnt_)
			{
				cv_.wait(ulk, [this] {return !cnt_;} );
				cnt_ = 0;
			}
		}
	public:
		helper():cnt_(0) {};
	};

	// member function

	pm(uint32_t t_num=0):
		work_(new boost::asio::io_service::work(service_))
	{

		uint32_t hard_num = boost::thread::hardware_concurrency();
		t_num = t_num >0?t_num:(hard_num > 1? hard_num-1:1);

		for(uint32_t i=0; i<hard_num; i++)
		{
			threadpool_.create_thread(
				boost::bind(&boost::asio::io_service::run, &service_)
			);
		}

#ifdef LQH_DEBUG
		std::cout << "Hardware support thread(s): " << hard_num << std::endl
				  << "PM start thread(s): " << t_num << std::endl;
#endif

	}

	~pm()
	{
		work_ = nullptr;
		threadpool_.join_all();
	}

	template <typename F>
	void post(helper& h, F f)
	{
		auto handler = [f, &h]()
		{
			f();
			h.decrease();
		};
		h.increase();
		service_.post(handler);
	}

	void wait(helper& h)
	{
		h.wait();
	}

};


class color
{
private:

public:
	using rgb = std::array<uint8_t,3>;
	using rgbs = std::vector<rgb>;
	static rgb HSL2RGB(double H, double S, double L)
	{
		rgb res{{0,0,0}};

		H = H<0 || H>360 ? 180:H;
		S = S<0 || S>1 ? 0.5:S;
		L = L<0 || L>1 ? 0.5:L;


		double C = (1-std::abs(2*L-1))*S;
		double X = C*(1-std::abs(std::fmod(H/60, 2) -1));
		double m = L - C/2;

		double RR, GG, BB;

		if(H <60)
		{
			RR = C;
			GG = X;
			BB = 0;
		}
		else if(H <120)
		{
			RR = X;
			GG = C;
			BB = 0;
		}
		else if(H <180)
		{
			RR = 0;
			GG = C;
			BB = X;
		}
		else if(H <240)
		{
			RR = 0;
			GG = X;
			BB = C;
		}
		else if(H <300)
		{
			RR = X;
			GG = 0;
			BB = C;
		}
		else
		{
			RR = X;
			GG = 0;
			BB = C;
		}

		res[0] = static_cast<uint8_t>((RR+m)*255);
		res[1] = static_cast<uint8_t>((GG+m)*255);
		res[2] = static_cast<uint8_t>((BB+m)*255);
		return res;
	};
	static rgbs get_rgbs(uint32_t n, double S=0.5, double L=0.5 )
	{
		n = !n ? 1:n;  // at least 1
		rgbs res(n);
		double step = 360/n;
		double H = 0;
		for(uint32_t i=0; i<n; i++, H+= step)
		{
			res[i] = HSL2RGB(H,S,L);
		}

		return res;
	}

};

}
}



#endif /* !UTILS_H */
