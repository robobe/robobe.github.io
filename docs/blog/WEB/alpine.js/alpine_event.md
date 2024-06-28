---
tags:
    - alpine.js
    - web
    - javascript
---

# Alpine.js Custom event


!!! note 
    custom event name must be lower latter without underline or other symbols 
     

```html title="" linenums="1" hl_lines="8"
<html>
<head>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/alpinejs/2.7.3/alpine.js"></script>
    <script>
        function trigger() {

            document.getElementById('alpine_root').dispatchEvent(
                new CustomEvent('triggerevent', { bubbles: true }));

        }

        const data = {
            index: 5,
            trigger() {
                this.index++;
                console.log(this.index);
            },
        }
    </script>
</head>

<body>
    <div id="alpine_root" 
        x-data="data"
        @triggerevent.window="trigger()">

    </div>
    <button onclick="trigger()">trigger</button>
</body>
```

## Demo 2
- bind event to setInterval method
- 
```html title="" linenums="1" hl_lines="8"
<html>
<head>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/alpinejs/2.7.3/alpine.js"></script>
    <script>
        function trigger() {

            document.getElementById('alpine_root').dispatchEvent(
                new CustomEvent('triggerevent', { bubbles: true }));

        }

        const data = {
            disabled: true,
            index: 5,
            trigger() {
                if (this.disabled){
                    this.index++;
                    console.log(this.index);
                }
            },
        }
    </script>
</head>

<body>
    <div id="alpine_root" 
        x-data="data"
        @triggerevent.window="trigger()">
        <input type="checkbox" x-model="disabled">
    </div>
    <button onclick="trigger()">trigger</button>
    <script>
        // setInterval(trigger, 1000);
        setInterval(function(){
            trigger();
        },1000)
    </script>
</body>
```

---

### Demo3

```html
<html>

<head>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/alpinejs/2.7.3/alpine.js"></script>
    <script>
        counter = 0;
        function trigger(counter) {
            document.getElementById('notification').dispatchEvent(
                new CustomEvent('notice', { detail: { text: counter }, bubbles: true }));

        }
        data = {
            add(notice) {
                console.log(notice)
            },
        }
    </script>
</head>


<body>
    <div id="notification" x-data="data" @notice.window="add($event.detail.text)">

    </div>
    <script>
        counter = 0;
        setInterval(() => {
            trigger(counter++);
        }, 1000);
    </script>
</body>
```