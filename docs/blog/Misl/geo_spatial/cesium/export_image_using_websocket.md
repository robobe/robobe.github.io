---
tags:
    - geo
    - spatial
    - cesium
    - image
    - websocket
    - camera
---

!!! tip "webgl preserveDrawingBuffer: true"
     By enabling preserveDrawingBuffer, you ensure that the CesiumJS canvas retains its last frame
     ```js
     const viewer = new Cesium.Viewer('cesiumContainer', {
            terrain: Cesium.Terrain.fromWorldTerrain(),
            contextOptions: {
                webgl: {
                    preserveDrawingBuffer: true // Enable preserveDrawingBuffer
                }
            }
        });
     ```


## canvas
- toDataURL
- toBlob
- getImageData: raw pixel data


---

## javascript modules

```js title="js/my_math.js"
export function add(a, b) {
    return a + b;
}

export function subtract(a, b) {
    return a - b;
}
```

import exported methods from `my_math.js` file
```js title="index.js"
import { add, subtract } from './js/my_math';

console.log(add(2, 3)); // Output: 5
```

---