#!/bin/bash

# Run this script to generate the documentation into a static website and deploy it to github pages.

# mkdocs expects a README.md placed in the docs/ folder as the primary page.
# No way to configure mkdocs to use the root readme so we temporary copy it to the right place.
cp README.md docs/
sed -i 's/](docs\//](/g' docs/README.md
mkdocs gh-deploy --clean
rm docs/README.md