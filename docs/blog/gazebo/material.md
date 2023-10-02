---
tags:
    - gazebo
    - sdf
    - material
---

!!! warning ""
     Gazebo does not support Ogre material files like Classic does, because Gazebo Gazebo can be used with multiple rendering engines. Therefore, materials defined within a `<script>` aren't supported on Gazebo, for example: 

    [more](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/migration_sdf.md#materials)

    ```xml
    <material>
        <script>
        <uri>model://number1/materials/scripts</uri>
        <uri>model://number1/materials/textures</uri>
        <name>Number/One</name>
        </script>
    </material>
    ```

```xml
<visual name="visual">
    <geometry>
        <sphere>
            <radius>1.0</radius>
        </sphere>
    </geometry>
    <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
        <specular>1 0 0 1</specular>
    </material>
</visual>
```

---

## Reference
- [Migration from Gazebo classic: SDF](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/migration_sdf.md)
- [Setting the material properties of visuals in SDFormat](http://sdformat.org/tutorials?tut=spec_materials)