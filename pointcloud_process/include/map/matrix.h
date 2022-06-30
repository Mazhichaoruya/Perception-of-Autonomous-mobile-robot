#ifndef _MATRIX_H
#define _MATRIX_H


#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define MAP_TYPE int
class matrix
{
private:
	int Col_;           //矩阵的行
	int Row_;            //矩阵的列
	int Size_;           //矩阵元素的数目
	MAP_TYPE* Elem_ = NULL;  //矩阵指针，根据需要分配大小（请查看构造函数)
public:
	void create(int Cols, int Rows, MAP_TYPE Val = 0);   //按行列数构造矩阵,默认值元素值为0
	void GetMapFromMap(matrix& raw_map,matrix& new_map,int start_row,int end_row,int start_col,int end_col);
	MAP_TYPE GetElem(int L,int R);             //查看第L行第R列元素的值
	void SetElem(int L, int R, MAP_TYPE val);        //修改第L行第R列元素的值
	int GetCols() { return Col_; };                //获取行数
	int GetRows() { return Row_; };                //获取列数
	int GetSize() { return Row_ * Col_; };   //获取数组大小
	bool IsInitialized(){return Elem_ != NULL;};
	
	~matrix(void);

};

#endif