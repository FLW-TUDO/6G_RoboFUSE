import sys
import os
import yaml

def replace_namespace_in_yaml(file_path, namespace):
    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    
    yaml_str = yaml.dump(yaml_data)
    yaml_str = yaml_str.replace('<robot_namespace>', namespace)
    
    with open(file_path, 'w') as file:
        file.write(yaml_str)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: replace_namespace.py <file_path> <namespace>")
        sys.exit(1)
    
    file_path = sys.argv[1]
    namespace = sys.argv[2]
    replace_namespace_in_yaml(file_path, namespace)
