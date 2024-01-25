---
tags:
    - rest api
    - js
    - javascript
    - async
---

- Using `cherryPy` a minimal python web framework 
  - Note: fastapi and flak request newer python version
- Using javascript to call REST api

# REST

- GET: Retrieve data
- POST: Add new data 
- PUT: Update exist data
- DEL: Remove data

### GET
- Get list of nodes
- 
```html title="get"
 <script>
    async function run_get(){
        const endpoint = new URL(`http://nano:8080/nodes`);
        
        const response = await fetch(endpoint,{
            headers: {
                
            }
        });

        if (response.status === 404){
            alert("Request API not found");
        }
        const data = await response.json();
        console.log(`row 0: ${data[0].name}`);
    }
</script>
```

### GET (node)
- Get request node

```javascript
async function run_get_one(index){
    const endpoint = new URL(`http://nano:8080/nodes/node${index}`);
    
    try {
        const response = await fetch(endpoint, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json'
            }
        });
        if (response.status === 404){
            alert("Request API not found");
        }
        
        const data = await response.json();
        console.log(`row ${index}: ${data.name}`);

    } catch (error) {
        console.log(error)
    }
}
```

```python title="server side route"
dispatcher.connect(name='nodes',
                    route='/nodes/{name}',
                    action='get',
                    controller=NodesController(),
                    conditions={'method': ['GET']})
```

### POST
- Add New node

```javascript
async function run_add(){
    console.log("Add new");
    const endpoint = new URL(`http://nano:8080/nodes`);
    data = {
        "name": "new_name"
    }
    try {
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(data)
        });
        if (response.status === 404){
            alert("Request API not found");
        }
        if (response.status === 204){
            console.log("delete OK")
        }

    } catch (error) {
        console.log(error)
    }
}
```

```python title="server route"
dispatcher.connect(name='nodes',
                    route='/nodes',
                    action='add_node',
                    controller=NodesController(),
                    conditions={'method': ['POST']})
```

### PUT

```javascript
async function run_update(index){
    console.log("PUT");
    const endpoint = new URL(`http://nano:8080/nodes/node${index}`);
    data = {
        "name": "update_name"
    }
    try {
        const response = await fetch(endpoint, {
            method: 'PUT',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(data)
        });
        if (response.status === 404){
            alert("Request API not found");
        }
        if (response.status === 204){
            console.log("update OK")
        }

    } catch (error) {
        console.log(error)
    }
}
```

```python title="server route"
dispatcher.connect(name='nodes',
                    route='/nodes/{name}',
                    action='update_node',
                    controller=NodesController(),
                    conditions={'method': ['PUT']})
```

### DELETE

```javascript
async function run_del(index){
    console.log("DEL");
    const endpoint = new URL(`http://nano:8080/nodes/node${index}`);
    try {
        const response = await fetch(endpoint, {
            method: 'DELETE',
            headers: {
                'Content-Type': 'application/json'
            }
        });
        if (response.status === 404){
            alert("Request API not found");
        }
        if (response.status === 204){
            console.log("delete OK")
        }

    } catch (error) {
        console.log(error)
    }
}
```

```python title="server route"
dispatcher.connect(name='nodes',
                    route='/nodes/{name}',
                    action='delete_node',
                    controller=NodesController(),
                    conditions={'method': ['DELETE']})
```

!!! note code 204
    HTTP Server success code without content
     
---
## Server

```python
#!/usr/bin/env python

# pylint: disable=invalid-name
import json
import cherrypy
from cherrypy.lib import auth_basic  # noqa pylint: disable=unused-import
from cherrypy.process import plugins
import cherrypy_cors
import os

USERS = {
    'user': 'password',
}

sample_nodes = [
    'node1',
    'node2',
]


class NodesController: \
        # pylint: disable=too-few-public-methods

    """Controller for fictional "nodes" webservice APIs"""

    @cherrypy.tools.json_out()
    def get_all(self):
        """
        Handler for /nodes (GET)
        """
        return [{'name': name} for name in sample_nodes]

    @cherrypy.tools.json_out()
    def get(self, name):
        """
        Handler for /nodes/<name> (GET)
        """

        if name not in sample_nodes:
            raise cherrypy.HTTPError(404, f'Node \"{name}\" not found')

        return {'name': name}

    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def add_node(self):
        """
        Handler for /nodes (POST)
        """
        cherrypy.response.status = 204
        request_data = cherrypy.request.json
        print(request_data)
        return f""

    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def update_node(self, name):
        """
        Handler for /nodes/<name> (PUT)
        """

        if name not in sample_nodes:
            raise cherrypy.HTTPError(404, f'Node \"{name}\" not found')

        # Empty response (http status 204) for successful PUT request
        cherrypy.response.status = 204
        request_data = cherrypy.request.json
        print(request_data)
        print(name)
        return ''

    @cherrypy.tools.json_in()
    @cherrypy.tools.json_out()
    def delete_node(self, name): \
            # pylint: disable=unused-argument
        """
        Handler for /nodes/<name> (DELETE)
        """

        # TODO: handle DELETE here

        # Empty response (http status 204) for successful DELETE request
        cherrypy.response.status = 204

        return ''


def jsonify_error(status, message, traceback, version): \
        # pylint: disable=unused-argument

    """JSONify all CherryPy error responses (created by raising the
    cherrypy.HTTPError exception)
    """

    cherrypy.response.headers['Content-Type'] = 'application/json'
    response_body = json.dumps(
        {
            'error': {
                'http_status': status,
                'message': message,
            }
        })

    cherrypy.response.status = status

    return response_body


def validate_password(realm, username, password): \
        # pylint: disable=unused-argument
    """
    Simple password validation
    """
    return username in USERS and USERS[username] == password


if __name__ == '__main__':
    PATH = os.path.abspath(os.path.dirname(__file__))
    cherrypy_cors.install()

    dispatcher = cherrypy.dispatch.RoutesDispatcher()

    # /nodes (GET)
    dispatcher.connect(name='nodes',
                       route='/nodes',
                       action='get_all',
                       controller=NodesController(),
                       conditions={'method': ['GET']})

    # /nodes/{name} (GET)
    #
    # Request "/nodes/notfound" (GET) to test the 404 (not found) handler
    dispatcher.connect(name='nodes',
                       route='/nodes/{name}',
                       action='get',
                       controller=NodesController(),
                       conditions={'method': ['GET']})

    # /nodes/{name} (POST)
    dispatcher.connect(name='nodes',
                       route='/nodes',
                       action='add_node',
                       controller=NodesController(),
                       conditions={'method': ['POST']})

    # /nodes/{name} (PUT)
    dispatcher.connect(name='nodes',
                       route='/nodes/{name}',
                       action='update_node',
                       controller=NodesController(),
                       conditions={'method': ['PUT']})

    # /nodes/{name} (DELETE)
    dispatcher.connect(name='nodes',
                       route='/nodes/{name}',
                       action='delete_node',
                       controller=NodesController(),
                       conditions={'method': ['DELETE']})

    config = {
        '/': {
            'tools.staticdir.on': True,
            'tools.staticdir.dir': PATH,
            'tools.staticdir.index': 'test.html',
            'request.dispatch': dispatcher,
            'error_page.default': jsonify_error,
            'cors.expose.on': True,
            'tools.auth_basic.on': False,
            'tools.auth_basic.realm': 'localhost',
            'tools.auth_basic.checkpassword': validate_password,
        },
    }

    cherrypy.tree.mount(root=None, config=config)

    cherrypy.config.update({
        'server.socket_host': '0.0.0.0',
        'server.socket_port': 8080,
    })

    cherrypy.engine.start()
    cherrypy.engine.block()

```


!!! note Config static page
    ```
    'tools.staticdir.on': True,
    'tools.staticdir.dir': PATH,
    'tools.staticdir.index': 'test.html',
    ```
     

---
## Resource
- [cherrypy-rest](https://github.com/EmmEff/cherrypy-rest/tree/master)
- [Promises in JavaScript (+ Async Await, Fetch API Example)](https://youtu.be/4o2QimWwPZU)
- [Fetch API with Async/Await (GET, POST, PUT, DELETE)](https://www.youtube.com/watch?v=3yi0yvHgPfQ)
- [How to Easily Call APIs With Fetch and Async/Await in JavaScript](https://www.youtube.com/watch?v=1Okmw8ggD1Q)