/**
 * @file    Matrix.cpp
 * @brief   定长矩阵的单位化与高斯-约当求逆实现。
 * @details 所属模块：userCode/algorithms，配套声明见 Inc/Matrix.h。
 *          求逆思路：将增广矩阵 [a | I] 中的 a 通过初等行变换化为单位矩阵，
 *          同步对右侧 I 施加完全相同变换，最终右侧即为 a^{-1}（结果在 this）。
 *          为避免引入额外内存，行交换时借用第 0 行 mat[0][j] 作临时存储。
 * @note    所有循环下标从 1 到 m/n，见头文件关于 1-based 下标与 N_max 的说明。
 * @note    主元用 fabs(...) > 1e-10 判定非零；若某列找不到非零主元，
 *          说明矩阵奇异，直接返回 false，此时 this 的内容是部分变换结果，不可用。
 */
//
// Created by admin on 2023/11/21.
//
#include "matrix.h"

/**
 * @brief 将当前矩阵置为单位矩阵（主对角线为 1，其余为 0）。
 * @note  遍历 1..m 行、1..n 列；对非方阵则只在 i==j 的位置写 1，
 *        其余位置写 0，并不检查 m、n 是否相等。
 *        inv() 在求逆开始时调用本函数构造增广矩阵右半部分。
 */
void Matrix::eye()
{
    /* 函数内部流程步骤：
     * 步骤 1：外层循环 i = 1..m 逐行扫描（下标从 1 开始）。
     * 步骤 2：内层循环 j = 1..n 逐列扫描。
     * 步骤 3：若 i == j（主对角线）写 1，否则写 0，得到单位矩阵。
     * 步骤 4：循环结束后 this 即可作为 inv() 中增广矩阵的右半部分。
     */
    for (int i = 1; i <= m; i++)
    {
        for (int j = 1; j <= n; j++)
        {
            if (i == j)
                mat[i][j] = 1;
            else
                mat[i][j] = 0;
        }
    }
}
/**
 * @brief 用高斯-约当消元求方阵 a 的逆，结果写入 this。
 * @details 算法分两大阶段：
 *          1) 自上而下：对每列 i 在 i..m 行中找主元，必要时交换 a 与 this
 *             的整行，把主元归一化为 1，再用它消去下方所有行的第 i 列，
 *             使 a 变成单位上三角矩阵；
 *          2) 自下而上：从第 m 行往回消去上方各行第 i 列的元素，
 *             使 a 变成单位矩阵；同步变换后的 this 即 a^{-1}。
 *          两个阶段中，对 a 施加的每一种行变换都原样施加到 this 上。
 * @param a 待求逆的方阵副本（按值传递，故原矩阵不会被破坏）。
 * @return true 求逆成功；false 表示 a 非方阵或某列无非零主元（奇异矩阵）。
 * @note  行交换借用第 0 行 mat[0][j] 作临时变量，因此交换必须从第 1 列
 *        开始且两矩阵同步进行；主元判定阈值为 1e-10，接近奇异的病态矩阵
 *        可能产生较大数值误差。
 * @note  返回 false 时 this 已被部分修改，调用方必须检查返回值后再使用结果。
 */
bool Matrix::inv(Matrix a)
{
    /* 函数内部流程步骤：
     * 步骤 1：检查 a 是否为方阵，非方阵直接返回 false。
     * 步骤 2：把 this 的维度设为 a 的维度，并 eye() 初始化为单位矩阵，
     *         构成增广矩阵 [a | I] 的右半部分。
     * 步骤 3：自上而下遍历主列 i（注意 i <= m）：
     *         3.1 在 i..m 行中找 |a.mat[k][i]| > 1e-10 的主元；
     *         3.2 找不到说明矩阵奇异，返回 false；
     *         3.3 主元不在第 i 行时，用第 0 行作中转同步交换 a 与 this 的整行；
     *         3.4 取主元 b = a.mat[i][i]，把第 i 行两个矩阵都除以 b 归一化；
     *         3.5 对下方各行 j，取 b = -a.mat[j][i]，把第 i 行的 b 倍加到
     *             第 j 行（两个矩阵同步），消去该列下方元素。
     * 步骤 4：自下而上遍历 i = m..2，对上方各行 j 取 b = -a.mat[j][i]，
     *         把第 i 行的 b 倍加到 this 的第 j 行，消去上三角残余；
     *         并显式把 a.mat[j][i] 置 0。
     * 步骤 5：a 已化为单位矩阵，this 即为 a 的逆，返回 true。
     */
    if (a.n != a.m)
    {
        
        return false;
    }
    m = a.m; n = a.n;
    eye(); // Create an identity matrix. //创建单位矩阵

		// Perform top-down elementary row operations to transform `a.mat` into an upper-triangular matrix with unit diagonal.
    //下来进行自上而下的初等行变换，使得矩阵 a.mat 变成单位上三角矩阵
    for (int i = 1; i <= m; i++) // Note that here `i <= m`, which differs from the earlier upper-triangular transformation, for we must check whether the last row’s last-column entry becomes zero after triangularization. 
    {                         	//注意这里要 i<=m，和之前的上三角矩阵有不同，因为要判断最后一行化为上三角矩阵的最后一行最后一列元素是否为 0
        // Search for a non-zero entry in column `i`.
				//寻找第 i 列不为零的元素
        int k;
        for (k = i; k <= m; k++)
        {
            if (fabs(a.mat[k][i]) > 1e-10) // When this condition is met, the entry is considered non-zero. //满足这个条件时，认为这个元素不为0
                break;
        }
        if (k <= m) // This indicates that column `i` contains at least one non-zero element. //说明第 i 列有不为0的元素
        {
            if (k != i) // This indicates that `a.mat[i][i]` is zero and a row swap is required. //说明第 i 行 第 i 列元素为零，需要和其他行交换
            {
								// Swap all elements of row `i` and row `k`.
                //交换第 i 行和第 k 行所有元素
                for (int j = 1; j <= n; j++) // The swap must start from the first element; note that this differs from the previous triangularization procedure.//需从第一个元素交换，注意与之前化上三角矩阵不同
                {// Use `mat[0][j]` as a temporary variable to swap elements; both matrices must be swapped accordingly. //使用mat[0][j]作为中间变量交换元素,两个矩阵都要交换
                    a.mat[0][j] = a.mat[i][j]; a.mat[i][j] = a.mat[k][j]; a.mat[k][j] = a.mat[0][j];
                    mat[0][j] = mat[i][j]; mat[i][j] = mat[k][j]; mat[k][j] = mat[0][j];
                }
            }
            float b = a.mat[i][i];//Multiplier. //倍数
            // Normalize the main diagonal elements of `a.mat` to 1.
						//将矩阵 a.mat 的主对角线元素化为 1
            for (int j = 1; j <= n; j++) // Start from the first element. //从第一个元素开始
            {
                a.mat[i][j] /= b;
                mat[i][j] /= b;
            }
            for (int j = i + 1; j <= m; j++)
            {
								//Note that the factor would normally be `-a.mat[j][i] / a.mat[i][i]`; since `a.mat[i][i] = 1`, the division is unnecessary.
                //注意本来为 -a.mat[j][i]/a.mat[i][i],因为a.mat[i][i]等于 1，则不需要除它
                b = -a.mat[j][i];
                for (k = 1; k <= n; k++)
                {
                    a.mat[j][k] += b * a.mat[i][k]; // Add (b × row i) to row j.//第 i 行 b 倍加到第 j 行
                    mat[j][k] += b * mat[i][k];
                }
            }
        }
        else
        {
            
            return false;
        }
    }

		// Next, perform bottom-up row operations to reduce `a.mat` to the identity matrix.
    //下面进行自下而上的行变换，将 a.mat 矩阵化为单位矩阵
    for (int i = m; i > 1; i--)
    {
        for (int j = i - 1; j >= 1; j--)
        {
            float b = -a.mat[j][i];
            a.mat[j][i] = 0;  // In practice, elementary row operations are applied to eliminate this entry. //实际上是通过初等行变换将这个元素化为 0
            for (int k = 1; k <= n; k++)
            {
							// Apply the same elementary row operations to transform the matrix on the right-hand side.
							//通过相同的初等行变换来变换右边矩阵
                mat[j][k] += b * mat[i][k];
            }
        }
    }
    return true;
}



