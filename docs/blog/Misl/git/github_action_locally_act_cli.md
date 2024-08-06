---
tags:
    - github
    - actions
    - act
    - git
    - devops
---

# Run github action locally
GitHub Actions help automate tasks like building, testing, and deploying in your GitHub repository. 
With `act` CLI tool we can write and test the GitHub action locally.


```
curl https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash
```


## config github action
- Add `.github/workflows` folder to project root
- Add yaml file and declare jobs


### view
from project root

```
act -l
```

### example
Run job on local docker

```
act -j build -P rome_arm=rome/arm:build --pull=false
```


---

## Reference
- [How to Run GitHub Actions Locally Using the act CLI Tool](https://www.freecodecamp.org/news/how-to-run-github-actions-locally/)