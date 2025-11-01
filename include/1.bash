# 1. 替换 std::void_t → boost::void_t
sed -i 's/std::void_t/boost::void_t/g' gnuplot-iostream.h

# 2. 替换 std::enable_if_t<...> → typename std::enable_if<...>::type
# 注意：需分两次替换（处理带模板参数的情况）
sed -i 's/typename std::enable_if_t</typename std::enable_if</g' gnuplot-iostream.h
sed -i 's/>>::type/>>::type/g' gnuplot-iostream.h  # 修正语法（若有多余>）

# 3. 替换 std::is_same_v<...> → std::is_same<...>::value
sed -i 's/std::is_same_v</std::is_same</g' gnuplot-iostream.h
sed -i 's/>>/>::value/g' gnuplot-iostream.h

# 4. 替换 std::is_base_of_v<...> → std::is_base_of<...>::value
sed -i 's/std::is_base_of_v</std::is_base_of</g' gnuplot-iostream.h
sed -i 's/>>/>::value/g' gnuplot-iostream.h
