//
// Created by admin on 2023/11/21.
//

/**
 * @file    Matrix.h
 * @brief   定长浮点矩阵类型：单位化与求逆。
 * @details 所属模块：userCode/algorithms。为运动学/控制解算中的小型矩阵求逆
 *          提供轻量实现，算法为高斯-约当（Gauss-Jordan）消元，位于 Src/Matrix.cpp。
 *          用固定大小二维数组代替动态分配，适合资源受限的 STM32 环境。
 * @note    元素下标采用 1-based：有效数据位于 mat[1..m][1..n]；
 *          第 0 行在 inv() 中被临时当作行交换的中转缓冲区。
 * @warning N_max 为 10，而数组下标合法范围是 0..N_max-1；由于下标从 1 开始，
 *          实际可安全使用的 m、n 应不超过 N_max-1（即 9），否则越界访问。
 * @note    默认构造函数不初始化 m/n/mat，使用前必须调用带行列数的构造函数
 *          或 inv()，否则矩阵内容不确定。
 */

#ifndef CONTROL_FRAME_MAIN_MATRIX_H
#define CONTROL_FRAME_MAIN_MATRIX_H

/* cmath 提供 fabs 等浮点数学函数（求逆时用于判断主元是否接近 0） */
#include <cmath>



const int N_max=10;     // 矩阵存储上限：mat 为 N_max × N_max 的静态数组

/**
 * @class Matrix
 * @brief m × n 浮点矩阵，支持 eye() 单位化与 inv() 求逆。
 */
class Matrix
{

public:

    int m, n ;//行数和列数
    float mat[N_max][N_max];  //矩阵开始的元素

    /** @brief 默认构造：仅占位，不设置行列数也不清零元素（见文件头 @warning）。 */
    Matrix() {}
    /**
     * @brief 构造 mm × nn 的零矩阵。
     * @param mm 行数。
     * @param nn 列数。
     * @note  只清零下标 1..m、1..n 的有效区域，第 0 行/列内容保持不确定，
     *        它们在 inv() 中仅作为临时存储使用。
     */
    Matrix(int mm, int nn)
    {
        /* 函数内部流程步骤：
         * 步骤 1：把形参行列数保存到成员 m、n。
         * 步骤 2：用双层循环遍历 i=1..m、j=1..n，把有效区域 mat[i][j] 清零。
         * 步骤 3：第 0 行/列不处理（保留给 inv() 作行交换中转）。
         */
        m = mm; n = nn;
        for (int i = 1; i <= m; i++) {
            for (int j = 1; j <= n; j++) {
                mat[i][j] = 0;
            }
        }
    }

    /**
     * @brief 用按行优先排列的一维数组构造 mm × nn 矩阵。
     * @param mm  行数。
     * @param nn  列数。
     * @param ele 元素数组首地址，须至少包含 mm * nn 个 float，按行依次排列。
     */
    Matrix(int mm, int nn, float *ele)
    {
        /* 函数内部流程步骤：
         * 步骤 1：保存行列数 m=mm、n=nn，并令一维数组下标 k=0。
         * 步骤 2：双层循环按行优先遍历 i=1..m、j=1..n。
         * 步骤 3：每步取 ele[k] 写入 mat[i][j]，然后 k++ 指向下一个元素；
         *         循环结束后完成按行填充（要求 ele 至少含 m*n 个元素）。
         */
        m = mm; n = nn;
        int k=0;
        for (int i = 1; i <= m; i++) {
            for (int j = 1; j <= n; j++) {
                mat[i][j] = ele[k];
                ++k;
            }
        }
    }

    /** @brief 把当前矩阵置为单位矩阵：i == j 处为 1，其余为 0（要求 m、n 已设置）。 */
    void eye();//令矩阵行列下标相同时值为 1，不同时为 0
    /** @brief 打印矩阵元素，用于调试输出（实现若未启用则为空）。 */
    void Print();

    /**
     * @brief 求矩阵 a 的逆，结果写回当前对象（this）。
     * @param a 待求逆的方阵，按值传参，函数内部对副本做行变换，不影响实参。
     * @return true 表示求逆成功；false 表示 a 非方阵或出现全零主元（奇异矩阵）。
     * @note  先把 this 置为单位矩阵，再对 [a | I] 同步做初等行变换，
     *        当 a 化为单位矩阵时，this 即为 a 的逆矩阵。
     */
    bool inv(Matrix a);//求矩阵的逆
};


#endif //CONTROL_FRAME_MAIN_MATRIX_H
