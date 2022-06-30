#include "map/matrix.h"

//构造矩阵：按行列
void matrix::create(int Rows, int Cols, MAP_TYPE val)
{
	Row_ = Rows;
	Col_ = Cols;
	Size_ = Row_ * Col_;
	if(Elem_ !=NULL)
	{
		delete []Elem_; 
		Elem_ = NULL;
	}
	Elem_ = new MAP_TYPE [Row_ * Col_];
	for(int i = 0 ; i < Row_ * Col_; i++)
		Elem_[i] = val;
}
void matrix::GetMapFromMap(matrix& raw_map,matrix& new_map,int start_row=-1,int end_row=-1,int start_col=-1,int end_col=-1)
{
	if(start_row==-1 || start_col == -1 || end_row==-1 || end_col == -1)
	{
		if(!raw_map.IsInitialized())
		{
			std::cout<<"Raw Matrix is not initialized!"<<std::endl;
			return;
		}
		new_map.create(raw_map.GetRows(),raw_map.GetCols(),0);
		for(int i=0;i<raw_map.GetRows();i++)
			for(int j=0;j<raw_map.GetCols();j++)
				new_map.SetElem(i,j,raw_map.GetElem(i,j));
	}
	else
	{
		int num_row = end_row - start_row;
		int num_col = end_col - start_col;
        if (num_row<0)
            std::swap(end_row,start_row);
        if (num_col<0)
            std::swap(end_col,start_col);
		if(num_row<0||num_col<0)
		{
//			std::cout<<"wrong parameter!"<<std::endl;
//			return;
		}
		new_map.create(raw_map.GetRows(),raw_map.GetCols(),0);
		for(int i=start_row;i<end_row;i++)
			for(int j=start_col;j<end_col;j++)
				new_map.SetElem(i,j,raw_map.GetElem(i,j));

	}
}
//查看第L行第R列元素的值
MAP_TYPE matrix::GetElem(int L,int R)                 
{
    if(L>=Row_||R>=Col_||L<0||R<0){//解决 索引越界的崩溃问题
//        std::cout<<"out of range of map"<<std::endl;
        return -1;
    }
	return Elem_[L * Col_ + R];
}
//修改第L行第R列元素的值
void matrix::SetElem(int L, int R, MAP_TYPE val)         
{
    if(L>=Row_||R>=Col_||L<0||R<0){//解决 索引越界的崩溃问题
//        std::cout<<"out of range of map"<<std::endl;
        return;
    }
	Elem_[(L) * Col_ + (R)] = val;
}

matrix::~matrix(void)
{
	if(Elem_!= NULL)
		delete []Elem_;            //矩阵所在函数块运行结束时才释放
}