---
tags:
    - alpine.js
    - web
    - javascript
---

# Alpine.js interval function


```html
<html>
<head>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/alpinejs/2.7.3/alpine.js"></script>
    <script>
        const data = {
            index: 5, 
            intervalId: null,
            log_it: function(){
                console.log(this.index++)
            },
            resetTheInterval: function () { 
                console.log("reset");
                console.log(this.index);
                this.intervalId = setInterval(()=>this.log_it(),1000);
                }
        }
    </script>
</head>

<body>
    <div x-data='data' x-init="resetTheInterval()">
        <button x-on:click="index = 3; resetTheInterval()">Reset</button>
    </div>
</body>
```