### Update dependencies defined in packages' `packages.xml`

```bash
# Run in workspace root
rosdep install -i --os=ubuntu:jammy --from-path src --ignore-src -r --rosdistro rolling -y
```
