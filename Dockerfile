FROM python:3.12-slim

WORKDIR /app
COPY pyproject.toml README.md ./
COPY vlm ./vlm
RUN pip install --no-cache-dir .

ENV VLM_HOST=0.0.0.0
ENV VLM_PORT=8080
EXPOSE 8080
CMD ["python", "-m", "vlm.server"]
