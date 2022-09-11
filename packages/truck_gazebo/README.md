# truck_gazebo

| [**docs**](../../doc/README.md) | [**packages**](../README.md) |
|---------------------------------|------------------------------|


Colleciton of gazebo misc files (models, worlds and etc).

### Upload meshes
1. Install [dvc](https://dvc.org/doc/install), more docs [here](https://dvc.org/doc/command-reference).
2. Contact us to get `id` and `token` of our storage
3. Run following commands

```
dvc remote add --local main s3://the-lab-bucket/dvc
dvc remote modify --local main endpointurl "https://storage.yandexcloud.net"
dvc remote modify --local main access_key_id "$id"
dvc remote modify --local main secret_access_key "$token"
dvc remote modify --local main region "ru-central1"
dvc remote default --local main
dvc install
dvc pull
```