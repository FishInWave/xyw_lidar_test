#include <iostream>
#include <vector>
using namespace std;
class A
{
public:
    A():a(2),b(1){}
    void foo()
    {
        cout<<"A::foo() is called"<<endl;
    }
protected:
    int a;
    int b;
};
class B:public A
{
public:
    void foo()
    {
        cout<<"B::foo() is called"<<endl;
    }
    void print(){
        cout << a << " " << b  << endl;
    }
};
void print(A msg){
    msg.foo();
}
void print(B msg){
    msg.foo();
}
int main(void)
{
    A a;
    B b;
    b.print();
   return 0;
}