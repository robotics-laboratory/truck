## Dev environment inside docker container

1. Запускаем контейнер как демона (флаг -d) или в другом терминале
2. Получаем ip address контейнера: `docker container inspect truck-base -f "{{.NetworkSettings.IPAddress}}"`
3. Заходим туда по ssh: `ssh root@172.17.0.2 -p 2222`
4. Пользователь `root`, пароль `root`

Также там же можно открыть vscode через Remote SSH extension. Тогда у вас будет навигация по коду и suggest'ы.
