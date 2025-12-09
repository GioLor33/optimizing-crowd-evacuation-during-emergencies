# this file can be runned from the project-root folder with:
# python -m tests.env1

from environments.environment import Environment

def main():
    
    env = Environment(
        name="Test Environment",
        dimensions=(10, 10),
        walls=[[(1, 1), (1, 2)]],
        exits=[[(0, 4), (4, 0)]]
    )
    
    print(env)
    
if __name__ == "__main__":
    main()