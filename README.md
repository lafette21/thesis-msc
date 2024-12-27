# thesis-msc

## Compiling with Baldr and Conan

```bash
baldr -p $(realpath .) -b Release -DSUPPRESS_COMPILER_WERROR=1 -DCMAKE_PROJECT_TOP_LEVEL_INCLUDES=$(realpath ./conan_provider.cmake)
```
