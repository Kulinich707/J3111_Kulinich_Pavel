# Естественно импортируем наш любимый numpy
import numpy as np

# Используем функцию для подсчёта количества чисел делящихся на 3
def count_3(arr):
    return np.sum(arr % 3 == 0)

# Ввод чисел 
user_input = input("Введите числа через пробел: ")

# Преобразование строки в массив чисел
user_array = np.array(list(map(int, user_input.split())))

# Вызов функции и вывод результата
result = count_3(user_array)
print(f"Количество чисел, делящихся на 3: {result}")