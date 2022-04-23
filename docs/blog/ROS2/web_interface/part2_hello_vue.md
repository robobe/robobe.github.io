---
title: Part2 - hello vue
description: learn basic vue javascript frame work for using roslibjs more convenient
date: "2022-04-22"
banner: ../images/vue.png
tags:
    - vue
    - web
    - 101
---
Vue.js is an open-source MVVM framework


!!! Note
    Install [HTML Preview](https://marketplace.visualstudio.com/items?itemName=tht13.html-preview-vscode)  
    and change it's security permission to view vue html pages  
    ![](../images/html_preview_security.png)


## hello 
```html
<script type="importmap">
    {
      "imports": {
        "vue": "https://unpkg.com/vue@3/dist/vue.esm-browser.js"
      }
    }
</script>

<div id="app">{{ message }}</div>

<script type="module">
    import { createApp } from 'vue'

    createApp({
        data() {
            return {
                message: 'Hello Vue3!'
            }
        }
    }).mount('#app')
</script>
```

## Data
The variables in Vue.js are JavaScript style variables  
and support all the standard types that are available in vanilla JavaScript. 

These types include:

- String - Stores strings
- Number - Stores integers, floating point, and exponential notations
- Boolean - Stores ‘true’ or ‘false’
- Null - Represents null value
- Object - Stores a dictionary
- Array - Stores a list of variables

```html title="data"
<script type="importmap">
    {
      "imports": {
        "vue": "https://unpkg.com/vue@3/dist/vue.esm-browser.js"
      }
    }
</script>

<div id="app">
    <div>string: {{ name }}</div>
    <div>number: {{ count }}</div>
    <div>bool: {{ visible }}</div>
    <div>array: {{ todos[1] }}</div>
    <div>null: {{ error }}</div>
    <div>object: {{ object.foo }}</div>
</div>

<script type="module">
    import { createApp } from 'vue'

    createApp({
        data() {
            return {
                name: 'Bob',                    //string
                count: 0,                       //number
                visible: false,                 //boolean
                todos: ['task1', 'task2'],      //array
                error: null,                     //null
                object: {                       //object
                    foo: 'bar'
                }
            }
        }
    }).mount('#app')
</script>
```

## Method
The methods property in the Vue instance contains the definitions of all the functions that the Vue instance can perform.  
Function cat called by specific event or where needed

```html title="method"
<script type="importmap">
    {
      "imports": {
        "vue": "https://unpkg.com/vue@3/dist/vue.esm-browser.js"
      }
    }
</script>

<div id="app">{{get_name()}}</div>

<script type="module">
    import { createApp } from 'vue'

    createApp({
        data() {
            return {
                greeting: 'Hello world'
            }
        },
        methods: {
                get_name: function(){
                    return this.greeting;
                }
        }
    }).mount('#app')
</script>
```

!!! Note
    using `this` to refer `data` variables
---

## References
- [vue3](https://vuejs.org/guide/quick-start.html)