// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {
        tic();
        //tic_new();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    void tic_new()
    {
      begn = std::chrono::high_resolution_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000; // sleep 1s, output: 1000.12 ms
    }

    double toc_new(){
      over = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(over-begn);

      return time_span.count();

    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::high_resolution_clock::time_point begn, over;

};
