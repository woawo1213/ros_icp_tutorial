#include <pcl/PCLPointCloud2.h>
#include <iostream>
using namespace std;

int main(int argc, char**argv) 
{
    // 선언
    boost::shared_ptr<vector<int>> vec_ptr(new vector<int>());
    vector<int> data = {1, 2, 3, 4, 5};
    
    cout<<vec_ptr<<endl;
    // Output:
    // 0x55e3cc53ce70 (달라질 수 있음)
    
    // 문법: shared_ptr가 가리키는 객체에 데이터를 "복사"하겠다는 의미!
    *vec_ptr = data;
    for (int i = 0; i < vec_ptr->size(); ++i) 
    {
        cout << (*vec_ptr)[i] << ", "; //*로 읽어야함.
    }
    cout << endl;
    // Output:
    // 1, 2, 3, 4, 5,
    
    cout<<vec_ptr<<endl;
    // Output:
    // 0x55e3cc53ce70 (달라질 수 있으나, 12번 줄과 똑같음!)
    
    
    // 선언 2
    boost::shared_ptr<vector<int> > vec_ptr2;
    vec_ptr2.reset(new vector<int>(3, 5));
    for (int i = 0; i < vec_ptr2->size(); ++i)
    {
        cout << (*vec_ptr2)[i] << ", ";
    }
    cout << endl;
    // Output:
    // 5, 5, 5,

    return 0;
}