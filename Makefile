.PHONY: lint format-check test coverage

PYTHON_VENV := $(firstword $(wildcard venv/bin/python .venv/bin/python))
PYTHON ?= $(if $(PYTHON_VENV),$(PYTHON_VENV),python3)
PYTEST ?= $(PYTHON) -m pytest
COVERAGE ?= $(PYTHON) -m coverage
RUFF ?= $(PYTHON) -m ruff

lint:
	PYTHONDONTWRITEBYTECODE=1 $(RUFF) check .

format-check:
	PYTHONDONTWRITEBYTECODE=1 $(RUFF) format --check .

test:
	PYTHONDONTWRITEBYTECODE=1 $(PYTEST) tests

coverage:
	PYTHONDONTWRITEBYTECODE=1 $(COVERAGE) run -m pytest tests
	$(COVERAGE) report -m
