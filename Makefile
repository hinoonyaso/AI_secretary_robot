.PHONY: up down restart logs build health clean

up:
	docker compose up -d

down:
	docker compose down

restart:
	docker compose restart

logs:
	docker compose logs -f --tail=100

build:
	docker build -t jetrover/brain:latest -f docker/Dockerfile .

health:
	@echo "=== Brain Core ==="
	@docker exec rover_brain_core curl -s http://localhost:8080/health || echo "FAIL"
	@echo "\n=== LLM Service ==="
	@docker exec rover_llm curl -s http://localhost:11434/health || echo "FAIL"
	@echo "\n=== VLM Service ==="
	@docker exec rover_vlm curl -s http://localhost:8081/health || echo "FAIL"

clean:
	docker compose down -v
	docker image prune -f
