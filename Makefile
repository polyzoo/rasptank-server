.PHONY: test coverage

PYTHON ?= python3
PYTEST ?= $(PYTHON) -m pytest
COVERAGE ?= $(PYTHON) -m coverage

test:
	PYTHONDONTWRITEBYTECODE=1 $(PYTEST) tests

coverage:
	PYTHONDONTWRITEBYTECODE=1 $(COVERAGE) run -m pytest tests
	$(COVERAGE) report -m
