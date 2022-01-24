# Репозиторий модулей для операционной системы Zephyr OS.

## Структура репозитория

Структура файлов и папок аналогична структуре [репозитория дистрибутива Zephyr](https://docs.zephyrproject.org/2.7.0/application/index.html#source-tree-structure).

### Верхний уровень cодержит следующие важные файлы:

#### `CMakeLists.txt`

Файл верхнего уровня системы сборки CMake.

#### `Kconfig`

Файл верхнего уровня системы кофигурирования Kconfig.

### Также на верхнем уровне располагаются следующие папки:

#### `arch`

Платформо-зависимый код.
Каждая поддерживаемая процессорная архитектура (x86, ARM и т.д.) располагается в собственном подкаталог, и содежит следующие файлы:

* платформо-зависимая часть исходного кода ядра ОС;
* платформо-зависимые заголовочные файлы ядра ОС для привватных API.

#### `soc`

Код и конфигурационные файлы, зависимые от типа микроконтроллера (Nordic, STM32 и т.д.).

#### `boards`

Код и конфигурационные файлы, зависимые от платы с микроконтроллером.

#### `doc`

Файлы документации.

#### `drivers`

Код драйверов устройств.

#### `dts`

Исходные файлы деревьев устройств (device tree siurces), используемые для необнаружимых устройств на платах.

#### `include`

Заголовочные файлы всех публичных API, за исключением API в директории `lib`.

#### `kernel`

Платформо-независимая часть исходного кода ядра ОС.

#### `lib`

Код библиотек, включая минимальную стандартную библиотеку языка C.

#### `misc`

Прочие файлы, не подходящие для остальных папкок верхнего уровня.

#### `samples`

Примеры приложений.

#### `scripts`

Программы и другие файлы для сборки и тестирования приложений Zephyr.

#### `cmake`

Дополнительные скрипты сборки CMake.

#### `subsys`

Подсистемы Zephjyr OS, включающие в себя cтек USB, cетевые стеки, Bluetooth, файловые системы и т.д.

#### `tests`

Тестовый код и окружение.

#### `share`

Дополнительные платформо-независимые файлы.

## Стиль исходного кода

Для кода в данном репозитории требуется соответсвие [стилю Zephyr](https://docs.zephyrproject.org/2.7.0/contribute/index.html#coding-style):

В целом, необходимо следовать стилю [Linux kernel coding style](https://kernel.org/doc/html/latest/process/coding-style.html), со следующими исключениями:

* Добавляйте скобки для каждого тела `if`, `else`, `do`, `while`, `for` и
  `switch`, даже для блоков кода из одного оператора. Используйте флаг `--ignore BRACES`
  для настройки *checkpatch*.
* Используйте пробелы вместо табуляций для выравнивания комментариев после определений, где это необходимо.
* Используйте однострочные комментарии в стиле C89, `/*  */`. Однострочные комментарии в стиле C99, `//`, запрещены.
* Используйте `/**  */` для комментариев doxygen, необходимых для добавления в документацию.


Для проверки соответсвия стилю используется утилита `checkpatch` от ядра Linux.

`checkpatch` доступен в папке `scripts` репозитория дистрибутива Zephyr. To invoke it when committing
code, make the file *$ZEPHYR_BASE/.git/hooks/pre-commit* executable and edit
it to contain:

```bash
    #!/bin/sh
    set -e exec
    exec git diff --cached | ${ZEPHYR_BASE}/scripts/checkpatch.pl -
```

Instead of running `checkpatch` at each commit, you may prefer to run it only
before pushing on zephyr repo. To do this, make the file
*$ZEPHYR_BASE/.git/hooks/pre-push* executable and edit it to contain:

```bash

    #!/bin/sh
    remote="$1"
    url="$2"

    z40=0000000000000000000000000000000000000000

    echo "Run push hook"

    while read local_ref local_sha remote_ref remote_sha
    do
        args="$remote $url $local_ref $local_sha $remote_ref $remote_sha"
        exec ${ZEPHYR_BASE}/scripts/series-push-hook.sh $args
    done

    exit 0
```

If you want to override checkpatch verdict and push you branch despite reported
issues, you can add option `--no-verify` to the git push command.

A more complete alternative to this is using `check_compliance.py` script from
ci-tools repo.
