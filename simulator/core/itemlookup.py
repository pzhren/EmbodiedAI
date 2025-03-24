import csv

class ItemLookup:
    def __init__(self, csv_file_path):
        self.items = {}
        self.load_csv(csv_file_path)

    def load_csv(self, csv_file_path):
        with open(csv_file_path, mode='r', encoding='utf-8') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.items[row['id']] = row['main_category']

    def get_item_name_by_id(self, item_id):
        # return self.items.get(item_id, "Item not found.")
        item_id = max(item_id.split("_"), key=len)
        for id, name in self.items.items():
            if item_id in id:
                return name

    def get_item_id_by_name(self, item_name):
        for id, name in self.items.items():
            if name == item_name:
                return id
        return "Item not found."



