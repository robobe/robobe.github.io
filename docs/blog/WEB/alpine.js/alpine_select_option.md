---
tags:
    - alpine.js
    - select
    - option
---


## Minimal example

```html
<html>

<head>
    <script>
        const data = {
            selected: 2
        }


    </script>
</head>

<body>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/alpinejs/2.7.3/alpine.js"></script>
    <div x-data="data">

        <select x-model="selected">
            <option value="1">one</option>
            <option value="2">two</option>
            <option value="3">three</option>
        </select>
        <br />
        selected: <span x-text="selected"></span>
        <br />
        Presets:
        <button @click="selected=1">set one</button>
        <button @click="selected=2">set two</button>
        <button @click="selected=3">set three</button>
    </div>
</body>

</html>
```

---

### More
Using template to build option from list

```html
<html>
<head>
    <script>
        const data = {
            selected: 2,
            options:[
                {code: 1, name: "one"},
                {code: 2, name: "two"},
                {code: 3, name: "three"},
            ]
        }
    </script>
</head>

<body>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/alpinejs/2.7.3/alpine.js"></script>
    <div x-data="data">

        <select x-model="selected">
            <template x-for="option in options">
                <option :value="option.code" x-text="option.name" :selected="selected===option.code"/>
            </template>
        </select>
        <br />
        selected: <span x-text="selected"></span>
        <br />
        Presets:
        <button @click="selected=1">set one</button>
        <button @click="selected=2">set two</button>
        <button @click="selected=3">set three</button>
    </div>
</body>

</html>
```