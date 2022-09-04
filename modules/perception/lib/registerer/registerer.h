/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <map>
#include <string>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace lib {

// idea from boost any but make it more simple and don't use type_info.
// 从 boost 库借鉴而来的 Any 类实现，可表示任意的抽象产品
class Any {
 public:
  Any() : content_(NULL) {}

  template <typename ValueType>
  explicit Any(const ValueType &value)
      : content_(new Holder<ValueType>(value)) {}

  Any(const Any &other)
      : content_(other.content_ ? other.content_->Clone() : nullptr) {}

  ~Any() { delete content_; }

  template <typename ValueType>
  ValueType *AnyCast() {
    return content_ ? &(static_cast<Holder<ValueType> *>(content_)->held_)
                    : nullptr;
  }

 private:
  class PlaceHolder {
   public:
    virtual ~PlaceHolder() {}
    virtual PlaceHolder *Clone() const = 0;
  };

  template <typename ValueType>
  class Holder : public PlaceHolder {
   public:
    explicit Holder(const ValueType &value) : held_(value) {}
    virtual ~Holder() {}
    virtual PlaceHolder *Clone() const { return new Holder(held_); }

    ValueType held_;
  };

  PlaceHolder *content_;
};

// 可用于生产任意抽象产品的抽象工厂类
class ObjectFactory {
 public:
  ObjectFactory() {}
  virtual ~ObjectFactory() {}
  virtual Any NewInstance() { return Any(); }
  ObjectFactory(const ObjectFactory &) = delete;
  ObjectFactory &operator=(const ObjectFactory &) = delete;

 private:
};

typedef std::map<std::string, ObjectFactory *> FactoryMap;
typedef std::map<std::string, FactoryMap> BaseClassMap;
BaseClassMap &GlobalFactoryMap();

// 结合 BaseClassMap 和 FactoryMap 获取指定基类对应的所有派生类
bool GetRegisteredClasses(
    const std::string &base_class_name,
    std::vector<std::string> *registered_derived_classes_names);

}  // namespace lib
}  // namespace perception
}  // namespace apollo

// 客户端代码
// 为指定的抽象产品类生成一个注册器类，包含用于生产产品实例的静态方法
// 宏定义中，## 用于连接两个记号，# 用于将宏参数转换为字符串
#define PERCEPTION_REGISTER_REGISTERER(base_class)                    \
  class base_class##Registerer {                                      \
    typedef ::apollo::perception::lib::Any Any;                       \
    typedef ::apollo::perception::lib::FactoryMap FactoryMap;         \
                                                                      \
   public:                                                            \
    static base_class *GetInstanceByName(const ::std::string &name) { \
      FactoryMap &map =                                               \
          ::apollo::perception::lib::GlobalFactoryMap()[#base_class]; \
      FactoryMap::iterator iter = map.find(name);                     \
      if (iter == map.end()) {                                        \
        for (auto c : map) {                                          \
          AERROR << "Instance:" << c.first;                           \
        }                                                             \
        AERROR << "Get instance " << name << " failed.";              \
        return nullptr;                                               \
      }                                                               \
      Any object = iter->second->NewInstance();                       \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
    static std::vector<base_class *> GetAllInstances() {              \
      std::vector<base_class *> instances;                            \
      FactoryMap &map =                                               \
          ::apollo::perception::lib::GlobalFactoryMap()[#base_class]; \
      instances.reserve(map.size());                                  \
      for (auto item : map) {                                         \
        Any object = item.second->NewInstance();                      \
        instances.push_back(*(object.AnyCast<base_class *>()));       \
      }                                                               \
      return instances;                                               \
    }                                                                 \
    static const ::std::string GetUniqInstanceName() {                \
      FactoryMap &map =                                               \
          ::apollo::perception::lib::GlobalFactoryMap()[#base_class]; \
      CHECK_EQ(map.size(), 1) << map.size();                          \
      return map.begin()->first;                                      \
    }                                                                 \
    static base_class *GetUniqInstance() {                            \
      FactoryMap &map =                                               \
          ::apollo::perception::lib::GlobalFactoryMap()[#base_class]; \
      CHECK_EQ(map.size(), 1) << map.size();                          \
      Any object = map.begin()->second->NewInstance();                \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
    static bool IsValid(const ::std::string &name) {                  \
      FactoryMap &map =                                               \
          ::apollo::perception::lib::GlobalFactoryMap()[#base_class]; \
      return map.find(name) != map.end();                             \
    }                                                                 \
  };

// 具体工厂类
// 在 Perception 功能模块 .so 动态库文件加载期间，被 __attribute__((constructor))
// 修饰的函数会优先执行，创建具体产品类对应的具体工厂指针，并将其映射到相应的 FactoryMap 中
#define PERCEPTION_REGISTER_CLASS(clazz, name)                                \
  namespace {                                                                 \
  class ObjectFactory##name : public apollo::perception::lib::ObjectFactory { \
   public:                                                                    \
    virtual ~ObjectFactory##name() {}                                         \
    virtual ::apollo::perception::lib::Any NewInstance() {                    \
      return ::apollo::perception::lib::Any(new name());                      \
    }                                                                         \
  };                                                                          \
  __attribute__((constructor)) void RegisterFactory##name() {                 \
    ::apollo::perception::lib::FactoryMap &map =                              \
        ::apollo::perception::lib::GlobalFactoryMap()[#clazz];                \
    if (map.find(#name) == map.end()) map[#name] = new ObjectFactory##name(); \
  }                                                                           \
  }  // namespace
