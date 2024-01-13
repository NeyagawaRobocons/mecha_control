from flask import Flask, request

app = Flask(__name__)

@app.route('/')
def handle_request():
    action = request.args.get('action', 'none')
    # ここでアクションに基づいた処理を行う
    print(f"アクション: {action}")
    return f"アクション {action} を受け取りました。"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000)