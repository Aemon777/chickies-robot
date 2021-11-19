import yaml

with open("comports.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        
        print(data['GPS_PORT'])
    except yaml.YAMLError as exc:
        print(exc)