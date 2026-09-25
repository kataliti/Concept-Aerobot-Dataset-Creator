# Concept Aerobot Dataset Creator

Набор инструментов для создания датасета полётов БПЛА в связке ROS 2, PX4/MAVROS и Gazebo. Проект записывает эталонный полёт с двух камер, воспроизводит его с вариациями скорости, траектории и окружения, а затем помогает разметить временные сегменты полёта.

Проект не хранит тяжёлые данные внутри Git-репозитория. Исходные записи и сгенерированные копии находятся в соседнем каталоге `datasets`.

## Что входит в проект

| Файл | Назначение |
| --- | --- |
| `record.py` | Записывает видео передней и нижней камер, MAVROS odometry, мировую траекторию Gazebo и параметры запуска симулятора. |
| `control.py` | Открывает Tk-интерфейс ручного управления БПЛА в режиме MAVROS OFFBOARD. |
| `record_start.sh` | Запускает QGroundControl, `record.py`, `control.py` и RViz в отдельных окнах. |
| `replay.py` | Точно воспроизводит сохранённую траекторию и создаёт новую вариацию полёта. |
| `flight_batch_ui.py` | Web-интерфейс пакетной генерации вариаций для нескольких групп исходных полётов. |
| `run_flight_batch_ui.sh` | Подготавливает ROS-окружение и запускает пакетный интерфейс. |
| `annotate.py` | Web-интерфейс разметки одного полёта или одной группы его числовых копий. |
| `annotate_mass.py` | Web-интерфейс массовой разметки оригиналов из `safe` с переносом разметки на копии из `raw`. |
| `run_annotate_mass.sh` | Совместимый launcher массовой разметки. |
| `replay_batch_reference.py` | Контрольная копия текущей реализации `replay.py`. |

## Рабочая структура каталогов

Рекомендуемая структура сохраняет данные рядом с проектом, а не внутри него:

```text
/home/sed/Desktop/concept-vla/
├── Concept-Aerobot-Dataset-Creator/   # этот Git-репозиторий
├── scripts -> Concept-Aerobot-Dataset-Creator/
│                                      # алиас для совместимости со старыми путями
└── datasets/
    ├── safe/                          # проверенные оригинальные полёты
    │   ├── route_01/                  # произвольная группа/маршрут
    │   │   ├── flight-20260925-120000/
    │   │   └── flight-20260925-121500/
    │   └── route_02/
    └── raw/                           # новые записи и replay-копии
        ├── flight-20260925-120000-1/
        ├── flight-20260925-120000-2/
        └── _batches/                  # JSON-манифесты пакетных запусков
```

Создание каталогов и алиаса из корня репозитория:

```bash
mkdir -p ../datasets/raw ../datasets/safe
test -e ../scripts || ln -s "$(basename "$PWD")" ../scripts
```

Алиас `scripts` нужен текущей версии проекта: несколько launcher-скриптов и значений по умолчанию всё ещё обращаются к `/home/sed/Desktop/concept-vla/scripts`. Сами данные при этом остаются вне репозитория.

> Пути `/home/sed/Desktop/concept-vla`, `~/Firmware2`, путь к QGroundControl и конфигурация RViz сейчас заданы в исходниках. При установке в другое место измените пользовательские константы в начале Python-файлов и пути в shell-скриптах.

## Формат одного полёта

Каждая папка `flight-YYYYMMDD-HHMMSS[-N]` является самостоятельной единицей датасета:

```text
flight-20260925-120000/
├── forward/
│   ├── video.mp4
│   ├── timestamps.csv
│   └── ffmpeg.log
├── bottom/
│   ├── video.mp4
│   ├── timestamps.csv
│   └── ffmpeg.log
├── odom/
│   └── odom.csv
├── trajectory/
│   ├── trajectory.csv
│   ├── initial_state.json
│   └── summary.json
├── environment/
│   ├── launch_context.json
│   └── launch_command.txt
├── augmentation/                      # только у replay-копий
│   ├── coord_bias.json
│   ├── speed.json
│   ├── lighting.json                  # если менялось освещение
│   └── batch_run.json                 # если запуск выполнен через batch UI
└── annotations/                       # появляется после разметки
    ├── subprograms.json
    └── subprograms.csv
```

Обозначения имён:

- `flight-YYYYMMDD-HHMMSS` — оригинальная запись;
- `flight-YYYYMMDD-HHMMSS-1`, `-2`, ... — последовательные replay-копии этого оригинала;
- временная шкала всех CSV начинается с поля `timestamp_start`;
- `environment/launch_context.json` хранит найденную команду `ros2 launch` и нужен пакетному генератору для восстановления карты.

## Требования

Проект рассчитан на уже настроенное окружение симуляции:

- Linux с графической сессией;
- Python 3 с `tkinter`;
- ROS 2 и пакет `rclpy`;
- PX4/MAVROS с сообщениями `geometry_msgs`, `sensor_msgs`, `mavros_msgs`;
- Gazebo CLI `gz`;
- `ffmpeg` с кодеком `libx264`;
- пакет симуляции `aerobot_gz_sim` и собранный ROS workspace;
- браузер для replay, batch и annotation UI;
- NumPy — для поддерживаемых `replay.py` преобразований изображений.

Используемые по умолчанию интерфейсы:

```text
/uav1/camera
/uav1/camera_down
/uav1/mavros/local_position/pose
/uav1/mavros/state
/uav1/mavros/setpoint_position/local
/uav1/mavros/cmd/arming
/uav1/mavros/set_mode
```

Перед запуском скриптов активируйте ROS workspace, например:

```bash
source ~/Firmware2/install/setup.bash
```

## Полный рабочий процесс

```text
ROS 2 + Gazebo
      │
      ▼
запись оригинала ──► datasets/raw/flight-.../
      │ проверка и ручной перенос
      ▼
datasets/safe/<группа>/flight-.../
      │
      ├──► одиночный replay
      └──► пакетный replay ──► datasets/raw/flight-...-N/
                                      │
                                      ▼
                       разметка оригинала и копий
```

### 1. Запустить симуляцию

Сначала запустите мир через `ros2 launch`. `record.py` ищет активный процесс запуска и сохраняет его команду в датасет. Если одновременно работают несколько launch-процессов, приоритет получает самый новый процесс пакета `aerobot_gz_sim`.

Это важно для пакетного replay: без корректного `environment/launch_context.json` исходный мир нельзя автоматически восстановить.

### 2. Записать эталонный полёт

Комплексный запуск для текущей локальной конфигурации:

```bash
./record_start.sh
```

Или запустите компоненты вручную в разных терминалах:

```bash
python3 record.py
python3 control.py
```

Управление в окне `control.py`:

| Клавиша | Действие |
| --- | --- |
| `Enter` | включить OFFBOARD и ARM |
| `↑` / `↓` | движение вперёд / назад |
| `←` / `→` | движение влево / вправо |
| `Left Shift` / `Left Ctrl` | подъём / снижение |
| `Z` / `X` | поворот влево / вправо |

Управление записью в терминале `record.py`:

- `Space` — начать или завершить текущую запись;
- `Q` — завершить recorder;
- запись не начнётся, пока не получена odometry;
- результат создаётся в `../datasets/raw/flight-YYYYMMDD-HHMMSS`.

После завершения проверьте видео, CSV и metadata. Проверенный оригинал перенесите в тематическую папку `safe`:

```bash
mkdir -p ../datasets/safe/route_01
mv ../datasets/raw/flight-20260925-120000 ../datasets/safe/route_01/
```

Имена `route_01`, `route_02` условны: используйте карту, маршрут, эксперимент или другой удобный признак группировки.

### 3. Создать одну replay-копию

Симулятор и MAVROS должны быть запущены, а необходимые topics и services — доступны.

```bash
python3 replay.py \
  ../datasets/safe/route_01/flight-20260925-120000 \
  --flight-speed 1.10 \
  --coord-frame body \
  --coord-dx 0.05 \
  --coord-y-amp 0.04 \
  --coord-yaw-amp 1.5 \
  --lighting day_to_sunset
```

Новая запись получит первый свободный числовой суффикс и будет сохранена в `datasets/raw`, например `flight-20260925-120000-1`.

Основные возможности `replay.py`:

- изменение физической скорости полёта через `--flight-speed`;
- постоянные смещения `X/Y/Z/yaw`;
- плавные coordinate wobble-вариации;
- системы координат `body` и `world`;
- сценарии освещения `off`, `constant`, `day_to_sunset`, `day_to_night`, `cloud_pass`, `random`;
- интерактивный Replay Control Center через `--ui`;
- пользовательские keyframes освещения через `--lighting-json`.

Полный список параметров:

```bash
python3 replay.py --help
```

### 4. Сгенерировать набор вариаций

```bash
./run_flight_batch_ui.sh
```

По умолчанию интерфейс откроется на `http://127.0.0.1:8787/`. Для проверки UI без запуска ROS и replay:

```bash
./run_flight_batch_ui.sh --dry-run
```

В интерфейсе:

1. укажите родительскую папку, например `../datasets/safe/route_01`;
2. при необходимости укажите фактический путь к `replay.py` — `<путь-к-репозиторию>/replay.py`;
3. задайте количество копий, seed, способ семплирования и диапазоны вариаций;
4. сначала постройте план, затем запустите batch.

Для каждого источника batch-выполнение:

- восстанавливает сохранённую команду запуска симулятора;
- сохраняет исходные launch-параметры, меняя только `wall_style` и `floor_style`;
- распределяет два встроенных стиля окружения в пропорции 50/50;
- детерминированно семплирует скорость и параметры траектории;
- проверяет полноту результата;
- пишет общий манифест в `datasets/raw/_batches` и metadata запуска в `augmentation/batch_run.json`.

### 5. Разметить датасет

Для отдельного полёта:

```bash
python3 annotate.py \
  ../datasets/raw/flight-20260925-120000-1 \
  --mode single
```

Режим `group` размечает выбранный полёт и все находящиеся рядом числовые копии с тем же базовым именем. Временные границы пересчитываются с учётом `augmentation/speed.json`:

```bash
python3 annotate.py \
  ../datasets/raw/flight-20260925-120000-1 \
  --mode group
```

Для основного сценария `safe → raw` используйте массовый интерфейс:

```bash
python3 annotate_mass.py \
  --safe-root ../datasets/safe \
  --raw-root ../datasets/raw \
  --parent route_01
```

По умолчанию интерфейс откроется на `http://127.0.0.1:8765/`. Он позволяет:

- просматривать оригиналы по родительским папкам внутри `safe`;
- синхронно смотреть видео обеих камер;
- размечать подпрограммы, состояния и временные сегменты;
- сохранить ручную разметку в оригинал;
- перенести её на все найденные `flight-...-N` в `raw` с пересчётом времени и кадров;
- видеть несинхронизированные, устаревшие и неполные копии.

Настройки массового интерфейса сохраняются в `~/.config/concept-vla/annotate_mass.json`.

## Проверка результата

Минимально полный flight-каталог должен содержать непустые файлы:

```text
forward/video.mp4
bottom/video.mp4
odom/odom.csv
trajectory/trajectory.csv
environment/launch_context.json
```

Быстрая ручная проверка:

```bash
test -s ../datasets/raw/flight-20260925-120000-1/forward/video.mp4
test -s ../datasets/raw/flight-20260925-120000-1/bottom/video.mp4
test -s ../datasets/raw/flight-20260925-120000-1/odom/odom.csv
test -s ../datasets/raw/flight-20260925-120000-1/trajectory/trajectory.csv
```

Логи кодирования камер находятся в `forward/ffmpeg.log` и `bottom/ffmpeg.log`.

## Важные ограничения

- Инструменты рассчитаны на симуляцию: `control.py` и `replay.py` выполняют ARM/OFFBOARD-команды.
- Web-интерфейсы по умолчанию слушают только `127.0.0.1`; не публикуйте их наружу без дополнительной защиты.
- Не переименовывайте flight-каталоги произвольно: группировка и перенос разметки зависят от шаблона `flight-YYYYMMDD-HHMMSS[-N]`.
- Для batch replay каждый оригинал должен содержать корректный `environment/launch_context.json`.
- `run_annotate_mass.sh`, `record_start.sh` и некоторые значения по умолчанию сохраняют старую схему пути `.../concept-vla/scripts`; для неё предусмотрен алиас из раздела «Рабочая структура каталогов».
