---
tags:
    - fastapi
    - rest
    - uvicorn
---

## Install
```bash title="install"
pip install fastapi uvicorn
```

## Define the API Endpoints

```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/hello")
def hello(name: str = ""):
    return {"message": f"Hello {name}"}
```

```python
import uvicorn

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Test

```bash title="from cli"
curl http://localhost:8000/hello?name=john
```

### From browser 

---

## Reference
- [Building an API using FastAPI and Uvicorn](https://dev.to/blst-security/building-an-api-using-fastapi-and-uvicorn-3h79)

