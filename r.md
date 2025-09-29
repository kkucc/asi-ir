https://github.com/nvladimus/daoSPIM?tab=readme-ov-file
https://github.com/pjb7687/single
https://archive.scanimage.org/SI2019/Supported-Microscope-Hardware_28377190.html
https://www.asiimaging.com/docs/ms2000_operation
https://asiimaging.com/docs/tiger_control_panel
https://www.asiimaging.com/downloads/manuals/ASI-PZ-WK-Inverted-XY.pdf
https://www.asiimaging.com/products/stages/piezo-z-axis-stages/pz-2000ft-series-automated-stage-with-piezo-z-axis-top-plate/#tab-id-5


### Общий формат команд

Все команды отправляются на контроллер через последовательный порт (RS-232 или USB) в виде текстовых строк. Команда завершается символом возврата каретки (\<CR>\)


### 1. Базовые команды для управления движением


```
MOVE (или M)
```

Переместить столик в абсолютные координаты (в десятых долях микрона). `MOVE X=10000 Y=5000` — переместит столик в точку (X=1 мм, Y=0.5 мм)

```
MOVREL (или R)
```

Переместить столик на заданное расстояние относительно текущей позиции.  `MOVREL X=-1000` — сместит столик на 0.1 мм назад по оси X.

```
WHERE (или W)
```

Узнать текущие координаты столика. `WHERE X Y` — контроллер вернет текущие координаты X и Y.

```
ZERO (или Z)
```

Установить текущую позицию как начало координат (0,0). `ZERO`

```
SPEED (или S)
```

Установить максимальную скорость движения (в мм/с).`SPEED X=5 Y=5`

```
 STATUS (или /)
```

Проверить, выполняется ли в данный момент движение. Возвращает `B`(Busy), если движение есть, и `N` (Not Busy), если столик свободен.

```
HALT (или `\`)
```

Немедленно остановить любое движение.
