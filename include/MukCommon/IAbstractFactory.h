#pragma once

#include "muk_common_api.h"
#include "MukException.h"

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>

#include <boost/tti/has_member_function.hpp>

#include <map>
#include <string>
#include <functional>
#include <algorithm>
#include <memory>
#include <vector>
#include <type_traits>


namespace gris
{
  namespace muk
  {

    BOOST_TTI_HAS_MEMBER_FUNCTION(clone)
      
    template< typename KeyType, typename ProductType >
    class IAbstractFactory
    {
      public:        
        typedef std::unique_ptr<ProductType>                    ProductCreatorType;
        typedef std::function<ProductCreatorType(void)>         ProductCreatorFunction;
        typedef std::map<KeyType, ProductCreatorFunction>       ProductTypeMap;

      public:
        bool register_class(const KeyType& key, const ProductCreatorFunction& func)
        {
          LOG_LINE << "Registered class " << key;
          mProducts.insert(std::make_pair(key, func));
          return true;
        }

        bool has_registered(const KeyType& key) const
        {
          return std::any_of(mProducts.begin(), mProducts.end(), [&](const auto& keyFunctorPair) { return keyFunctorPair.first == key; });
        }

        ProductCreatorType create(const KeyType& key) const
        {          
          if (!has_registered(key))
          {
            throw MUK_EXCEPTION("Key was not registered in Factory", key.c_str());
          }
          auto ptr = mProducts.at(key)();
          return ptr;
        }

        /**
        * create a 
        */
        template<typename T = ProductType>
        typename std::enable_if<std::is_same<T, ProductType>::value && has_member_function_clone<T, void, boost::mpl::vector<const T*>>::value, ProductCreatorType>::type 
          create(const T* rhs) const
        {
          ProductCreatorType ret = create(rhs->name());
          ret->clone(rhs);
          return ret;
        }

        template<typename T = ProductType>
        typename std::enable_if<std::is_same<T, ProductType>::value && has_member_function_clone<T, void, boost::mpl::vector<const T*>>::value, ProductCreatorType>::type
          create(const T&& rhs) const
        { return create(&rhs); }

        template<typename T = ProductType>
        typename std::enable_if<std::is_same<T, ProductType>::value && has_member_function_clone<T, void, boost::mpl::vector<const T*>>::value, ProductCreatorType>::type
          create(const T& rhs) const
        { return create(&rhs); }

        template<typename T = ProductType>
        typename std::enable_if<std::is_same<T, ProductType>::value && has_member_function_clone<T, void, boost::mpl::vector<const T*>>::value, ProductCreatorType>::type
          create(const std::unique_ptr<T>& rhs) const
        { return create(rhs.get()); }

        template<typename T = ProductType>
        typename std::enable_if<std::is_same<T, ProductType>::value && has_member_function_clone<T, void, boost::mpl::vector<const T*>>::value, ProductCreatorType>::type
          create(const std::shared_ptr<T>& rhs) const
        { return create(rhs.get()); }

        virtual void getKeys(std::vector<std::string>& keys) const
        {
          std::for_each(mProducts.begin(), mProducts.end(), [&] (const std::pair<KeyType, ProductCreatorFunction>& pair) { keys.push_back(pair.first); });          
        }

      private:      
        ProductTypeMap mProducts;
    };
  }
}
