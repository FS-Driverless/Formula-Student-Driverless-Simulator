# How to release a new version

## Export the Unreal Engine project for release
1. Open the UE4Project in the Unreal Editor
3. Ensure 'File' -> 'Package Project' -> 'Build configuration' is set to 'Shipping,
2. Choose 'File' -> 'Package Project' -> 'Windows (64 bit)'
3. Select any folder on your computer.
4. Wait until it finishes.
5. Go into the `WindowsNoEditor` folder and rename `Blocks.exe` to `FSDS.exe`
6. Zip all files and upload to github release!

## Deploying documentation
For documentation we use [mkdocs](https://www.mkdocs.org/) and [mike](https://github.com/jimporter/mike/).
Hosting is provided by github pages. 

To tag a new version of the documentation and release it to github, first checkout the version that you want to deploy.
Then run `mike deploy VERSION latest -u -p`.
This will compile the documentation, store it as a new version in the `gh-pages` branch, update the `latest` alias to point at the new version and push the `gh-pages` branch to github and thus making the documentation public.
To create a new version without updating the `latest` tag, omit the `latest -u` part. 

