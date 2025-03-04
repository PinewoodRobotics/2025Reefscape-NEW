Orientations:

*_Look, I know this is wierd! This will have to be re-worked at a future date._

---

FIELD:
+x = right
-x = left
+y = forward
-y = back

```
              +y
              |
              |
 -x <---------|----------> +x
              |
              |
              -y
```

### Usage

Essentially this is the map orientation that will be rendered on the user's screen. Look for this when trying to get some user input. 

---

SWERVE:
+x = forward
-x = back
+y = left
-y = right

```
              +x
              |
              |
 +y <---------|----------> -y
              |
              |
              -x
```

### Usage

Swerve drive library relies on this coordinate system. Nothing more to say here.

---

GLOBAL:
-y = forward
+y = back
+x = left
-x = right

```
              -y
              |
              |
 +x <---------|----------> -x
              |
              |
              +y
```

### Usage

This is the orientation system that the april tag and position extrapolator subprocesses use in order to do their thing. 

#### Why so wierd?

Essentially, in april tag world, -z is actually forward so the y just represents the z here.

---