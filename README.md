# RVMSIS_PROJECT_CREATOR (Поддержка Visual Studio Code for Embedded v2.0). Поддержка Windows и Linux.
### Внимание! 
В Windows теперь не обязаельно создавать папку CMSIS только по пути C:\\. 
Скрипты научились работать с относительными путями. 
Единственное - избегайте кириллицу в пути. Также важен запуск скрипта от имени администратора Windows и sudo в Linux. 
Создавать ярлыки теперь тоже нет смысла, все работает из коробки. Просто запускаем *.bat файл (Windows) или *.sh файл (Linux) нужного проекта, он запросит запуститься от администратора и далее создаст копию Clean_project с новым названием, которое Вы пропишете в консоли.
P.S. На Linux при копировании папки RVMSIS с интернета возможно слетят настройки прав доступа к файлам и каталогам. Исправить это просто командой "chmod -R 777 RVMSIS" (при условии, что вы запустили скрипт в папке RVMSIS_PROJECT_CREATOR)

# Ниже будет старенькая инструкция по созданию проекта:
## Скрипты для создания нового проекта под RVMSIS/WCH (RISC-V)
### Инструкция для Windows: Копируем папку RVMSIS в корень диска C.
Чтоб создать чистый проект под RVMSIS со своим именем - нажимаем на интересующий Вас ярлык, например CH32V203C8T6_new_project.lnk
Вводим имя и жмем enter:
![image](https://github.com/Solderingironspb/RVMSIS_PROJECT_CREATOR/assets/68805120/fd350c3c-8878-4de9-a7be-bbab7349ff6d)
## Проект готов
![image](https://github.com/Solderingironspb/RVMSIS_PROJECT_CREATOR/assets/68805120/17a87dec-9336-4afd-9f8d-225fbac68a92)

## Открываем его в MounRiver Studio и пользуемся
![image](https://github.com/Solderingironspb/RVMSIS_PROJECT_CREATOR/assets/68805120/5c6e1c9b-877d-4ed1-a9e5-611a1c17120b)



