//
// Created by 25396 on 2023/2/11.
//

/**
 * @file    Map.h
 * @brief   基于二叉搜索树（BST）的极简键值映射容器 MyMap。
 * @details 所属模块：userCode/algorithms。为避免在 MCU 上引入 STL（std::map
 *          代码体积与动态分配开销较大）而实现的模板容器：每个键值对是一个
 *          由 new 分配的 Node，通过 left / right 指针构成二叉搜索树，
 *          Key 必须支持 < 与 > 比较。仅提供 insert、operator[]、find 三个操作，
 *          没有删除、没有自平衡，按键有序插入时树会退化为链表（最坏 O(n) 查找）。
 *          全部实现内联在头文件中，包含本头的每个编译单元各自完成模板实例化。
 * @note    节点只分配不释放（类无析构函数），容器应作为全局或长生命周期对象使用，
 *          不要在循环中反复创建/销毁，以免内存泄漏。
 * @warning 非线程安全，中断与主循环同时访问时需外部保护。
 */

#ifndef RM_FRAME_C_MAP_H
#define RM_FRAME_C_MAP_H


/**
 * @class MyMap
 * @brief 模板化二叉搜索树映射，接口模仿 std::map 的常用子集。
 * @tparam Key   键类型，需可比较大小。
 * @tparam Value 值类型，operator[] 要求可默认构造。
 */
template <typename Key, typename Value>
class MyMap {
public:
    /**
     * @struct Node
     * @brief 树节点：一个键值对加左右子指针。
     */
    struct Node {
        Key first;      // 键，BST 的排序依据
        Value second;   // 与键关联的值
        Node *left;     // 左子树：存放比 first 更小的键
        Node *right;    // 右子树：存放比 first 更大的键
    };

    /** @brief 构造空映射，根节点置空。 */
    MyMap() : root_(nullptr) {}

    /**
     * @brief 插入或覆盖一个键值对。
     * @param key   待插入的键。
     * @param value 待写入的值。
     * @note  从根节点向下查找插入位置：key 较小走左子树，较大走右子树，
     *        相等则原地更新 second 并直接返回（不新增节点，也不会有重复键）。
     *        新节点以 new 分配，左右指针均初始化为 nullptr。
     * @warning 树根为空时直接把新节点作为根；类不负责释放节点内存。
     */
    void insert(const Key &key, const Value &value) {
        /* 函数内部流程步骤：
         * 步骤 1：new 一个 Node，写入 key/value，左右子指针置空。
         * 步骤 2：若 root_ 为空，直接把新节点作为根并返回。
         * 步骤 3：否则从 root_ 开始沿树下降：key 小于当前键走左子树，
         *         大于走右子树；子树为空时挂上新节点并返回。
         * 步骤 4：若 key 与当前节点相等，说明键已存在，
         *         只更新 second（不新增节点、不产生重复键）后返回。
         * 步骤 5：循环直到插入完成；新节点由调用方/长期容器持有，类不释放。
         */
        Node *node = new Node;
        node->first = key;
        node->second = value;
        node->left = nullptr;
        node->right = nullptr;

        if (!root_) {
            root_ = node;
            return;
        }

        Node *cur = root_;
        while (cur) {
            if (key < cur->first) {
                if (cur->left) {
                    cur = cur->left;
                } else {
                    cur->left = node;
                    return;
                }
            } else if (key > cur->first) {
                if (cur->right) {
                    cur = cur->right;
                } else {
                    cur->right = node;
                    return;
                }
            } else {
                cur->second = value;
                return;
            }
        }
    }

    /**
     * @brief 下标访问：返回键对应值的可写引用；键不存在时自动插入默认值。
     * @param key 要访问的键。
     * @return 对应节点 second 成员的引用，可用于读写。
     * @note  找不到键时先 insert(key, Value()) 插入默认构造的值，再递归调用
     *        operator[] 取回引用；因此 Value 必须可默认构造，且本操作可能因
     *        new 分配内存，不宜在高频/中断上下文中调用。
     */
    Value& operator[](const Key &key) {
        /* 函数内部流程步骤：
         * 步骤 1：从 root_ 开始沿 BST 下降查找 key。
         * 步骤 2：key 小于当前键走左子树，大于走右子树。
         * 步骤 3：相等则立即返回该节点 second 的可写引用。
         * 步骤 4：走到空子节点说明键不存在，先 insert(key, Value())
         *         插入一个默认构造值（可能触发 new）。
         * 步骤 5：递归调用 operator[](key) 重新查找并返回新节点引用。
         */
        Node *node = root_;
        while (node) {
            if (key < node->first) {
                node = node->left;
            } else if (key > node->first) {
                node = node->right;
            } else {
                return node->second;
            }
        }

        insert(key, Value());
        return operator[](key);
    }

    /**
     * @brief 判断键是否存在（只读，不插入节点）。
     * @param key 要查找的键。
     * @return 找到返回 true；遍历到空子节点返回 false。
     * @note  与 operator[] 的区别是不会改变树结构，适合仅做存在性判断的场合。
     */
    bool find(const Key &key) {
        /* 函数内部流程步骤：
         * 步骤 1：从 root_ 开始循环比较。
         * 步骤 2：key 小于当前键走左子树，大于走右子树。
         * 步骤 3：相等立即返回 true（不修改树）。
         * 步骤 4：走到空指针说明不存在，循环结束返回 false。
         */
        Node *node = root_;
        while (node) {
            if (key < node->first) {
                node = node->left;
            } else if (key > node->first) {
                node = node->right;
            } else {
                return true;
            }
        }
        return false;
    }

    Node* root_;    // 二叉搜索树根节点，public 便于外部直接遍历（也意味着无封装保护）
};

#endif //RM_FRAME_C_MAP_H
