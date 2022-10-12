---
title: Part3 - vue user handling
description: learn basic vue user handling
date: "2022-04-22"
banner: ../images/vue.png
tags:
    - vue
    - binding
    - 101
---

- live server
- vetur
- material icon theme
## Basic struct
- index.html: contain vue root element
- app.js: create app and mount to the root element


```html title="index.html"
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>vue</title>
    <script src="https://unpkg.com/vue@3.0.2"></script>
</head>
<body>
    <!--template-->
    <div id="app">
        <p>{{ title }}</p>
    </div>

    <script src="app.js"></script>
</body>
</html>
```

```js title="app.js"
const app = Vue.createApp({
    data() { 
            return {
                title: "hello vue"    
            }
    }
})
app.mount('#app')
```

---

## Bind and event

```html title="vue div"
<div id="app">
    <p>{{ title }} {{counter}}</p>
    <button v-on:click="counter++">inc</button>
    <button @click="counter--">dec</button>
    <div @click="counter=0">reset</div>
</div>
```

!!! Note
    `@` is a shorthand for `v-on`  
    `v-on:click` is `@click`

### methods property
- Add `methods` property to vue app

```html
<div id="app">
    <p>{{ title }} {{counter}}</p>
    <button @click="reset">reset</button>
    <button @click="preset(10)">preset</button>
</div>
```

```js
const app = Vue.createApp({
    data() { 
            return {
                title: "hello vue",
                counter: 0
            }
    },
    methods: {
        reset(){
            this.counter = 0
        },
        preset(value){
            this.counter = value
        }
    }
})
```