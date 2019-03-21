# jetson_car
Набор разного ПО для экспериментов с моделью беспилотного автомобиля

## TL;NR КАК ЗАПУСТИТЬ
Пришел очередной губернатор, надо срочно пустить пыль в глаза, а Маркова под рукой нет? 

1. [Настроить](https://github.com/Garrus007/jetson_car/tree/master/jetson_car#%D0%BC%D0%B8%D0%BD%D0%B8-%D0%B3%D0%B0%D0%B9%D0%B4). Если никто ничего не ломал, это уже настроено.
2. Jetson
```sh
$ roslaunch jetson_car path_move_jetson.launch obstacles:=true path_file:=<absolute path>
```
3. Host, визуализация
```sh
$ jetson_ros
$ roslaunch jetson_car path_move_host.launch
```

## Реализовано
 - управление двигателями платформы с помощью NVidia Jetson
 - удаленное телеуправление с помощью джойстика (клиент под ROS и Windows)
 - экспериментальное движение по заданной траектории

## Аппаратная платформа
- Платформа: 6-колесная мобильная платформа от Pololu
- Бортовой компьютер: NVidia Jetson TX2
- ZED camera
- Драйвер двигателей: Pololu md03a
- STM32F103C8T6 для связи NVidia Jetson с драйвером

<img src="docs/car.png"/>

[Подробнее об аппаратной платформе](HARDWARE.md)

## Состав репозитория
### RCWin32
Программа для телеуправления с помощью джойстика для Windows (.NET Framework). На текущий момент пока не поддерживается ROS-пакетом.

## stm32_firmware
Прошивка для микроконтроллера соединительной платы

## jetson_car
Пакет ROS с основной функциональностью. Пакет общий и для NVidia Jetson, и для ПК оператора, чтобы можно было выбирать, где запускать какие компоненты. 

[Подробнее о пакете jetson_car](jetson_car/README.md)
