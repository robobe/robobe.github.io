import yaml

if __name__ == '__main__':
    yaml_file = "/home/user/projects/blog/examples/python/data/data.yaml"
    stream = open(yaml_file, 'r')
    dictionary = yaml.load(stream)
    print(dictionary)