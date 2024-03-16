#include <iostream>
#include <vector>
#include <algorithm>


template <typename Container, typename MemberType>
class PropertyIterator {
public:
    typedef typename Container::value_type ValueType;
    typedef typename Container::iterator Iterator;

    PropertyIterator(Container& container, MemberType ValueType::*memberPtr)
        : container(container), memberPtr(memberPtr), current(container.begin()) {}

    PropertyIterator& operator++() {
        ++current;
        return *this;
    }

    PropertyIterator operator++(int) {
        PropertyIterator temp = *this;
        ++(*this);
        return temp;
    }

    MemberType& operator*() const {
        return ((*current).*memberPtr);
    }

    bool operator!=(const PropertyIterator& other) const {
        return current != other.current;
    }

    // 添加begin成员函数
    PropertyIterator begin() const {
        return *this;
    }

    // 添加end成员函数
    PropertyIterator end() const {
        return PropertyIterator(container, memberPtr, container.end());
    }

private:
    PropertyIterator(Container& container, MemberType ValueType::*memberPtr, Iterator cur)
        : container(container), memberPtr(memberPtr), current(cur) {}

    Container& container;
    MemberType ValueType::*memberPtr;
    Iterator current;
};

// struct Person {
//     std::string name;
//     int age;
// };
// int main() {
//     std::vector<Person> people = {{"Alice", 25}, {"Bob", 30}, {"Charlie", 22}};

//     // 使用begin和end
//     for (const auto& age : PropertyIterator<std::vector<Person>, int>(people, &Person::age)) {
//         std::cout << age << " ";
//     }

//     std::cout << std::endl;

//     return 0;
// }