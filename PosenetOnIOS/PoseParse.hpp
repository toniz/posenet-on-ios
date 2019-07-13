//
//  PoseParse.hpp
//  PosenetOnIOS
//
//  Created by toniz sin on 9/7/2019.
//

#ifndef PoseParse_hpp
#define PoseParse_hpp

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#define FLOAT_TO_INT 10000
#define INT_TO_FLOAT 10000.0

using namespace std;

class C3DArray
{
public:
    C3DArray(int h1, int w1, int s1){
        h = h1;
        w = w1;
        s = s1;
    };
    ~C3DArray(){};
    
    inline int getOffset(int y, int x, int o){
        return (y * w + x) * s + o;
    }

public:
    int h, w, s;
};

class CDecodePose
{
public:
    CDecodePose();
    ~CDecodePose(){};
    bool isMaxScoreInLocalWindow(int y, int x, int k, float* score, int &iScore);
    bool isInResultNmsRadius(int h, int w, int k, map<int, map<int, vector<int>>> &result);
    bool BuildTopQueue(float *pScore, map<float, vector<int>> &topQueue);
    bool findNextKeyPoint(int isForward, int edge, float* pScore,float *pShortOffset, float *pMiddleOffset, map<int, vector<int>> &keyPoints, int &scoreSum);
    int decode(float *score, float *s_offset, float *m_offset, map<int, map<int, vector<int>>> &result);

public:
    int m_inputHeight;
    int m_inputWidth;
    int m_yCnt;
    int m_xCnt;
    int m_kCnt;
    int m_eCnt;
    
    vector<string> m_keyPointName;
    vector<int> m_childOrder;
    vector<int> m_parentOrder;
    
private:
    C3DArray* m_scoreArray;
    C3DArray* m_shortOffsetArray;
    C3DArray* m_middleOffsetArray;
    
    int m_pointThreshold;
    int m_localWindowRadius;
    int m_outStride;
    int m_nmsRadius;
    int m_squaredNmsRadius;
};

#endif /* PoseParse_hpp */
