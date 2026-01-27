#ifndef TEMPLATE_CLASS_H_
#define TEMPLATE_CLASS_H_

#include <iostream>

//单例基类，用于快速构造一个单例类
template<typename T>
class CSingletonTemp
{
public:
    //外部访问时，必须以引用的方式访问
    static T& GetInstance()
    {
        static T instance;
        return instance;
    }
    virtual ~CSingletonTemp(){ /*std::cout << "destructor called!" << std::endl;*/ }
    CSingletonTemp(const CSingletonTemp&)=delete;
    CSingletonTemp& operator =(const CSingletonTemp&)=delete;
protected:
    CSingletonTemp(){ /*std::cout << "constructor called!" << std::endl;*/ }
};

//单例类使用示例
class  CDerivedSingle : public CSingletonTemp<CDerivedSingle>
{
    //需要将基类声明成友员，以便访问子类的私有构造函数和析构函数
   friend class CSingletonTemp<CDerivedSingle>;
public:
    CDerivedSingle(const  CDerivedSingle&)=delete;
    CDerivedSingle& operator =(const  CDerivedSingle&)= delete;
   ~ CDerivedSingle(){}
private:
    CDerivedSingle(){}
};
#endif //!TEMPLATE_CLASS_H_
