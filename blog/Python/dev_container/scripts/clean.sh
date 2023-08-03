rm -rf deb_dist
rm -rf dist
find . | grep -E "(/__pycache__$|\.pyc$|\.pyo$)" | xargs rm -rf
rm -rf .mypy_cache
rm -rf .pytest_cache
rm -rf app.egg-info
rm app*.tar.gz
rm -rf build
