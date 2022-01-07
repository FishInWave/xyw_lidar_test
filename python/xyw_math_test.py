#!/usr/bin/python3
# coding=utf-8 

import numpy as np
# 检查矩阵是否正定
def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) > 0)

# numpy的使用以及cov
# cov的分母是n-1
# 列为观测，行为变量。对于三维点来说，每一列都应该是三个，每一行应该有n个数据，下面用了转置，所以逻辑相反。
A = np.array([[1,3,5],[-2,-3,-4],[-1,-3,-5]])
B = np.array([[1,2,3],[2,3,5],[0,0,1]])
print("A rank: ", np.linalg.matrix_rank(A))
print(A*A.T)
print(np.linalg.matrix_rank(A*A.T))
print(np.linalg.eigvals(A*A.T))

cov_A = np.cov(A.T)
cov_B = np.cov(B.T)
print("A is posi ?" , is_pos_def(cov_A))
print("B is posi ?",is_pos_def(cov_B))
print("cov determinant: " , np.linalg.det(cov_A))
# new_x = x+1e-9*np.identity(3)
new_x = cov_A+1e-9*cov_B
print("cov + aI determinant: " , np.linalg.det(new_x))
print(is_pos_def(new_x))
print(np.zeros((3,3)))


