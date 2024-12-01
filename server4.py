from flask import Flask, request, jsonify
import csv

app = Flask(__name__)
csv_file = 'measurements.csv'

@app.route('/submit', methods=['POST'])
def submit_data():
    print("Headers:", request.headers)
    print("Data:", request.get_data(as_text=True))
    
    if request.is_json:
        data = request.get_json()
        print("Received data:", data)

        # Validate if all required fields are present
        required_fields = ['raw_distance', 'average_distance', 'r_value', 'q_value']
        if all(field in data for field in required_fields):
            write_to_csv(data['raw_distance'], data['average_distance'], data['r_value'],data['q_value'])
            return jsonify({"message": "Data received successfully"}), 200
        else:
            missing = [field for field in required_fields if field not in data]
            return jsonify({"error": "Missing data in request", "missing_fields": missing}), 400
    else:
        return jsonify({"error": "Request must be JSON", "content_type": request.headers.get('Content-Type')}), 400

def write_to_csv(raw_distance, average_distance, r_value,q_value):
    with open(csv_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([raw_distance, average_distance, r_value,q_value])

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
