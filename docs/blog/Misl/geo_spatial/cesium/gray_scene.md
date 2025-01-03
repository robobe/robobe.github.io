---
tags:
    - geo
    - spatial
    - cesium
    - image
    - websocket
    - camera
---


```js
const grayscaleShader = new Cesium.PostProcessStage({
    fragmentShader: `
        #version 300 es
        precision highp float;

        // Declare the input texture coordinate from the vertex shader
        in vec2 v_textureCoordinates;

        // Declare the output color
        out vec4 fragColor;

        // Sampler2D for the color texture
        uniform sampler2D colorTexture;

        void main() {
            // Get the color from the texture
            vec4 color = texture(colorTexture, v_textureCoordinates);

            // Convert the color to grayscale using the luminance formula
            float gray = 0.3 * color.r + 0.59 * color.g + 0.11 * color.b;

            // Set the color to grayscale
            fragColor = vec4(gray, gray, gray, color.a);
        }
    `
});

viewer.scene.postProcessStages.add(grayscaleShader);
```