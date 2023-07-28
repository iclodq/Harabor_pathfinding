#pragma once

#include <cassert>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ostream>
#include <string>
struct Statistic
{
public:
    uint64_t addOpenCnt = 0;    // openlist节点添加次数
    uint64_t closeCnt   = 0;    // close数量
    uint64_t reopenCnt   = 0;   // 重新加入次数
    uint64_t maxOpenCnt = 0;    // open节点最大数量
    uint64_t uptOpenCnt = 0;    // 更新open节点
    uint64_t expandCnt  = 0;    // 扩展点数量
    uint64_t touchedCnt = 0;    

    uint64_t findJumpCnt = 0;
    uint64_t cardinalCnt = 0;    // 直线方向 主方向
    uint64_t diagonalCnt = 0;    // 斜线方向 分方向

    uint64_t callCardinalCnt = 0;
    uint64_t callDiagonalCnt = 0;
    uint64_t callFindJumpCnt = 0;

    std::chrono::high_resolution_clock::time_point timerStart;
    int64_t                               costTime = 0;

    void Clear()
    {
        addOpenCnt = 0;
        closeCnt   = 0;
        reopenCnt   = 0;
        maxOpenCnt = 0;
        uptOpenCnt = 0;
        expandCnt  = 0;
        touchedCnt = 0;

        findJumpCnt = 0;
        cardinalCnt = 0;
        diagonalCnt = 0;

        callCardinalCnt = 0;
        callDiagonalCnt = 0;
        callFindJumpCnt = 0;
    }
    void TimerStart()
    {
        timerStart = std::chrono::high_resolution_clock::now();
    }
    void TimerStop()
    {
        costTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - timerStart).count();
    }
    friend std::ostream& operator<<(std::ostream& os, const Statistic& d)
    {
#define OutAttr(X) #X ": " << d.X << "\n"
        os << "Statistic:\n"
           << OutAttr(addOpenCnt)
           << OutAttr(reopenCnt)
           << OutAttr(closeCnt)
           << OutAttr(maxOpenCnt)
           << OutAttr(uptOpenCnt)
           << OutAttr(expandCnt)
           << OutAttr(touchedCnt)
           << OutAttr(findJumpCnt)
           << OutAttr(cardinalCnt)
           << OutAttr(diagonalCnt)
           << OutAttr(callFindJumpCnt)
           << OutAttr(callCardinalCnt)
           << OutAttr(callDiagonalCnt)
           << OutAttr(costTime);

#undef OutAttr

        // os << "OptOpenCnt:" << d.addOpenCnt << "\n"
        //    << "WhileCnt:" << d.whileCnt << "\n"
        //    << "MaxOpenCnt:" << d.maxOpenCnt << "\n"
        //   << "CardinalCnt:" << d.cardinalCnt << "\n"
        //   << "DiagonalCnt:" << d.diagonalCnt << "\n"
        //   << "FindJumpCnt:" << d.findJumpCnt << "\n";
        return os;
    }
};
extern Statistic g_statistic;
#define STATISTIC(x) ++_statistic.x
#define STATISTIC_CLEAR() _statistic.Clear()
